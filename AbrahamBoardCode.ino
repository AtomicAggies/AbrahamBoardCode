#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <TelemetryData.h>

const int chipSelect = BUILTIN_SDCARD;

const uint8_t I2C_RECEIVE_ADDRESS = 0x00;
const uint8_t I2C_FRAME_MAX_SIZE = 32;
const uint8_t I2C_FRAME_HEADER_SIZE = 2;
const uint8_t I2C_FRAME_PAYLOAD_SIZE = I2C_FRAME_MAX_SIZE - I2C_FRAME_HEADER_SIZE;

// I2C frame header byte 0:
// bit 0: frame is intended for the SD-card receiver
// bit 1: frame is intended for the radio/antenna receiver
// bit 7: frame starts a new telemetry packet; clear means continuation
// I2C frame header byte 1 is an 8-bit checksum of the payload bytes that
// follow the header.
const uint8_t I2C_FRAME_DESTINATION_SD = 1 << 0;
const uint8_t I2C_FRAME_START = 1 << 7;

const uint8_t FRAME_QUEUE_SIZE = 8;
const unsigned long PACKET_RECEIVE_TIMEOUT_MS = 1000;

struct I2CFrame {
  uint8_t length;
  uint8_t bytes[I2C_FRAME_MAX_SIZE];
};

volatile uint8_t frameQueueHead = 0;
volatile uint8_t frameQueueTail = 0;
volatile uint16_t droppedFrameCount = 0;
I2CFrame frameQueue[FRAME_QUEUE_SIZE];

uint8_t telemetryBuffer[TELEMETRY_PACKET_MAX_BYTES];
uint8_t telemetryBufferLength = 0;
uint8_t expectedPacketBytes = 0;
bool receivingPacket = false;
unsigned long lastPacketFrameMillis = 0;

bool sdReady = false;
uint32_t validPacketCount = 0;
uint32_t invalidPacketCount = 0;
uint32_t ignoredFrameCount = 0;
uint32_t checksumFailureCount = 0;
String logFilename = "current-log.txt";
String binaryFilename = "current-data.bin";

uint8_t checksumI2CPayload(const uint8_t *payload, uint8_t payloadSize) {
  uint8_t checksum = 0;
  for (uint8_t index = 0; index < payloadSize; index++) {
    checksum += payload[index];
  } 
  return checksum;
}

void appendLog(const String &message) {
  if (!sdReady || logFilename.length() == 0) {
    return;
  }

  File logFile = SD.open(logFilename.c_str(), FILE_WRITE);
  if (!logFile) {
    return;
  }

  logFile.print(millis());
  logFile.print(" ms: ");
  logFile.println(message);
  logFile.close();
}

void discardPartialPacket(const String &reason) {
  if (receivingPacket || telemetryBufferLength > 0) {
    invalidPacketCount++;
    String expectStr =
        expectedPacketBytes > 0 ? String(expectedPacketBytes) : String("?");
    appendLog("Discarded partial telemetry packet (" + String(telemetryBufferLength) +
              "/" + expectStr + " bytes): " + reason);
  }

  telemetryBufferLength = 0;
  expectedPacketBytes = 0;
  receivingPacket = false;
  lastPacketFrameMillis = 0;
}

void writeTelemetryPacket() {
  if (!sdReady || binaryFilename.length() == 0) {
    return;
  }

  File binaryFile = SD.open(binaryFilename.c_str(), FILE_WRITE);
  if (!binaryFile) {
    appendLog("ERROR: could not open " + binaryFilename + " for telemetry write");
    return;
  }

  uint8_t recordBytes = telemetryBuffer[0];
  if (recordBytes < 2 || recordBytes > TELEMETRY_PACKET_MAX_BYTES) {
    invalidPacketCount++;
    appendLog("ERROR: invalid wireLength for SD write");
    return;
  }
  if (recordBytes != telemetryBufferLength) {
    invalidPacketCount++;
    appendLog("ERROR: wireLength does not match assembled length");
    return;
  }

  size_t bytesWritten = binaryFile.write(telemetryBuffer, recordBytes);
  binaryFile.close();

  if (bytesWritten == recordBytes) {
    validPacketCount++;
    appendLog("Wrote telemetry packet " + String(validPacketCount) + " to " + binaryFilename +
              " (" + String(recordBytes) + " bytes)");
  } else {
    invalidPacketCount++;
    appendLog("ERROR: short SD write for telemetry packet (" + String(bytesWritten) +
              "/" + String(recordBytes) + " bytes written)");
  }
}

void processI2CFrame(const I2CFrame &frame) {
  if (frame.length < I2C_FRAME_HEADER_SIZE) {
    invalidPacketCount++;
    appendLog("Discarded invalid I2C frame shorter than header: " + String(frame.length) + " bytes");
    discardPartialPacket("short I2C frame");
    return;
  }

  uint8_t frameFlags = frame.bytes[0];
  uint8_t receivedChecksum = frame.bytes[1];
  uint8_t payloadSize = frame.length - I2C_FRAME_HEADER_SIZE;
  const uint8_t *payload = frame.bytes + I2C_FRAME_HEADER_SIZE;
  bool isStartFrame = (frameFlags & I2C_FRAME_START) != 0;

  if ((frameFlags & I2C_FRAME_DESTINATION_SD) == 0) {
    ignoredFrameCount++;
    return;
  }

  uint8_t calculatedChecksum = checksumI2CPayload(payload, payloadSize);
  if (calculatedChecksum != receivedChecksum) {
    checksumFailureCount++;
    appendLog("Checksum failure on I2C frame (received 0x" + String(receivedChecksum, HEX) +
              ", calculated 0x" + String(calculatedChecksum, HEX) + ")");
    discardPartialPacket("I2C frame checksum failure");
    return;
  }

  if (isStartFrame) {
    discardPartialPacket("new telemetry packet started before previous packet was complete");
    telemetryBufferLength = 0;
    expectedPacketBytes = 0;
    receivingPacket = true;
    lastPacketFrameMillis = millis();
  } else if (!receivingPacket) {
    invalidPacketCount++;
    appendLog("Discarded continuation I2C frame with no active telemetry packet");
    return;
  }

  if (payloadSize == 0) {
    discardPartialPacket("empty I2C frame payload");
    return;
  }

  if (telemetryBufferLength + payloadSize > TELEMETRY_PACKET_MAX_BYTES) {
    discardPartialPacket("I2C frame would overflow telemetry buffer");
    return;
  }

  memcpy(telemetryBuffer + telemetryBufferLength, payload, payloadSize);
  telemetryBufferLength += payloadSize;
  lastPacketFrameMillis = millis();

  if (telemetryBufferLength >= 1 && expectedPacketBytes == 0) {
    expectedPacketBytes = telemetryBuffer[0];
    if (expectedPacketBytes < 2 ||
        expectedPacketBytes > TELEMETRY_PACKET_MAX_BYTES) {
      discardPartialPacket("invalid wireLength in telemetry header");
      return;
    }
  }

  if (expectedPacketBytes > 0 &&
      telemetryBufferLength > expectedPacketBytes) {
    discardPartialPacket("received more bytes than wireLength");
    return;
  }

  String targetStr =
      expectedPacketBytes > 0 ? String(expectedPacketBytes) : String("?");
  appendLog("Accepted I2C frame payload (" + String(payloadSize) + " bytes, " +
            String(telemetryBufferLength) + "/" + targetStr +
            " packet bytes buffered)");

  if (expectedPacketBytes > 0 &&
      telemetryBufferLength == expectedPacketBytes) {
    writeTelemetryPacket();
    telemetryBufferLength = 0;
    expectedPacketBytes = 0;
    receivingPacket = false;
    lastPacketFrameMillis = 0;
  }
}

bool popQueuedFrame(I2CFrame &frame) {
  noInterrupts();
  if (frameQueueHead == frameQueueTail) {
    interrupts();
    return false;
  }

  frame = frameQueue[frameQueueTail];
  frameQueueTail = (frameQueueTail + 1) % FRAME_QUEUE_SIZE;
  interrupts();
  return true;
}

String archivedFilename(const String &currentFilename, unsigned short fileNumber) {
  int dotIndex = currentFilename.lastIndexOf('.');
  String baseName;
  String extension;

  if (dotIndex >= 0) {
    baseName = currentFilename.substring(0, dotIndex);
    extension = currentFilename.substring(dotIndex);
  } else {
    baseName = currentFilename;
    extension = "";
  }

  if (baseName.startsWith("current-")) {
    baseName = baseName.substring(String("current-").length());
  }

  return baseName + "-" + String(fileNumber) + extension;
}

bool rotateCurrentFile(const String &currentFilename) {
  if (!SD.exists(currentFilename.c_str())) {
    return true;
  }

  for (unsigned short fileNumber = 1; fileNumber < 10000; fileNumber++) {
    String archiveFilename = archivedFilename(currentFilename, fileNumber);
    if (!SD.exists(archiveFilename.c_str())) {
      return SD.rename(currentFilename.c_str(), archiveFilename.c_str());
    }
  }

  return false;
}

void clearCurrentFile(const String &currentFilename) {
  if (SD.exists(currentFilename.c_str())) {
    SD.remove(currentFilename.c_str());
  }

  File currentFile = SD.open(currentFilename.c_str(), FILE_WRITE);
  if (currentFile) {
    currentFile.close();
  }
}

void createLogFiles() {
  bool logRotated = rotateCurrentFile(logFilename);
  bool binaryRotated = rotateCurrentFile(binaryFilename);

  clearCurrentFile(logFilename);
  clearCurrentFile(binaryFilename);

  File logFile = SD.open(logFilename.c_str(), FILE_WRITE);
  if (logFile) {
    logFile.println("SD-card telemetry receiver booted");
    logFile.close();
  }

  if (!logRotated) {
    appendLog("WARNING: could not archive previous " + logFilename);
  }

  if (!binaryRotated) {
    appendLog("WARNING: could not archive previous " + binaryFilename);
  }
}


void setup() {
  Wire.begin(I2C_RECEIVE_ADDRESS);
  Wire.onReceive(receiveEvent);

  sdReady = SD.begin(chipSelect);
  if (!sdReady) {
    return;
  }

  createLogFiles();
  appendLog("Logging to " + logFilename + " and raw telemetry to " + binaryFilename);
  appendLog("Listening for length-prefixed telemetry (max " +
            String(TELEMETRY_PACKET_MAX_BYTES) +
            " bytes/record) on I2C address 0x" +
            String(I2C_RECEIVE_ADDRESS, HEX));
}

void loop() {
  static uint16_t lastDroppedFrameCount = 0;

  noInterrupts();
  uint16_t currentDroppedFrameCount = droppedFrameCount;
  interrupts();

  if (currentDroppedFrameCount != lastDroppedFrameCount) {
    uint16_t droppedSinceLastLog = currentDroppedFrameCount - lastDroppedFrameCount;
    lastDroppedFrameCount = currentDroppedFrameCount;
    appendLog("WARNING: I2C frame queue overflow dropped " + String(droppedSinceLastLog) + " frame(s)");
    discardPartialPacket("I2C frame queue overflow");
  }

  I2CFrame frame;
  while (popQueuedFrame(frame)) {
    processI2CFrame(frame);
  }

  if (receivingPacket && lastPacketFrameMillis != 0 &&
      millis() - lastPacketFrameMillis > PACKET_RECEIVE_TIMEOUT_MS) {
    discardPartialPacket("timed out before full telemetry packet received");
  }
}

void receiveEvent(int count) {
  if (count > I2C_FRAME_MAX_SIZE) {
    while (Wire.available()) {
      Wire.read();
    }
    droppedFrameCount++;
    return;
  }

  uint8_t nextHead = (frameQueueHead + 1) % FRAME_QUEUE_SIZE;
  if (nextHead == frameQueueTail) {
    while (Wire.available()) {
      Wire.read();
    }
    droppedFrameCount++;
    return;
  }

  I2CFrame &frame = frameQueue[frameQueueHead];
  frame.length = 0;

  while (Wire.available()) {
    uint8_t byteValue = Wire.read();
    if (frame.length < I2C_FRAME_MAX_SIZE) {
      frame.bytes[frame.length++] = byteValue;
    } else {
      droppedFrameCount++;
    }
  }

  frameQueueHead = nextHead;
}
