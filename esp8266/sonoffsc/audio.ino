
// WAV header spec information:
//https://web.archive.org/web/20140327141505/https://ccrma.stanford.edu/courses/422/projects/WaveFormat/
//http://www.topherlee.com/software/pcm-tut-wavformat.html

typedef struct wav_header {
    // RIFF Header
    char riff_header[4]; // Contains "RIFF"
    int wav_size; // Size of the wav portion of the file, which follows the first 8 bytes. File size - 8
    char wave_header[4]; // Contains "WAVE"

    // Format Header
    char fmt_header[4]; // Contains "fmt " (includes trailing space)
    int fmt_chunk_size; // Should be 16 for PCM
    short audio_format; // Should be 1 for PCM. 3 for IEEE Float
    short num_channels;
    int sample_rate;
    int byte_rate; // Number of bytes per second. sample_rate * num_channels * Bytes Per Sample
    short sample_alignment; // num_channels * Bytes Per Sample
    short bit_depth; // Number of bits per sample

    // Data
    char data_header[4]; // Contains "data"
    int data_bytes; // Number of bytes in data. Number of samples * num_channels * sample byte size
    // uint8_t bytes[]; // Remainder of wave file is bytes
} wav_header;

#define WAVSIZE (256)

char dummyHeader[] = {
  0x52, 0x49, 0x46, 0x46, 0x24, 0xdf, 0x01, 0x00, 0x57, 0x41, 0x56, 0x45, 0x66, 0x6d, 0x74, 0x20, 0x10, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x80, 0x3e, 0x00, 0x00, 0x80, 0x3e, 0x00,
  0x00, 0x01, 0x00, 0x08, 0x00, 0x64, 0x61, 0x74, 0x61, 0x00, 0xdf, 0x01, 0x00, 0x77, 0x84, 0x8b, 0x96, 0x8f, 0x96, 0x8c, 0x78, 0x7a, 0x84, 0x81, 0x7b, 0x72, 0x7b, 0x88, 0x8e, 0x79, 0x6b,
  0x5e, 0x83, 0x88, 0x85, 0x6f, 0x7f, 0x7d, 0x7d, 0x7c, 0x7a, 0x68, 0x81, 0x8d, 0x8f, 0x81, 0x87, 0x79, 0x8c, 0x89, 0x8b, 0x8b, 0x82, 0x88, 0x8a, 0x8f, 0x85, 0x89, 0x76, 0x80, 0x7c, 0x7f
};

typedef struct wavData {
  struct wav_header header;
  int16_t data[WAVSIZE];
} wavData;

float calculateNoiseDb(int16_t * data, size_t size) {
  float sumSampleSquared = 0;
  for (int i = 0 ; i < size ; i++) {
    float normalized = ((float)data[i] - 128) / 128;
    sumSampleSquared += normalized * normalized;
  }
  if (0) {
  Serial.print("squaredSum: ");
  Serial.println(sumSampleSquared);
  }
  float rms = sqrt (sumSampleSquared / size);

  if (0) {
  Serial.print("RMS value : ");
  Serial.println(sumSampleSquared);
  }
  float noiseDb = 20 * log10 (rms);

  if (0) {
  Serial.print("NoiseDb  : ");
  Serial.println(noiseDb);
  }
  return noiseDb;
}

extern float noise;

struct wavData audioFrame;
size_t audioFrameCounter;

bool audioLoadData(char * data, size_t size) {

  if (size != 64) {
    Serial.print("bad requested size 64 != ");
    Serial.println(size);
  }

  for (int i = 0 ; i < size && audioFrameCounter < WAVSIZE ; i++) {
    int16_t v = ((unsigned char *)data)[i];
    v = v - 128;
    v = v << 8;

    audioFrame.data[audioFrameCounter++] = v;
  }

  if (0) {
  Serial.print("audioFrameCounter: ");
  Serial.println(audioFrameCounter);
  }
  if(audioFrameCounter >= WAVSIZE) {
    audioFrameCounter = 0;

    noise = calculateNoiseDb(&audioFrame.data[0], WAVSIZE);

    mqttSendByteStream("wav", (const char *)&audioFrame.data[0], WAVSIZE*2);

    memcpy(&audioFrame.header, &dummyHeader[0], sizeof(audioFrame.header));

    audioFrame.header.wav_size = sizeof(audioFrame) - 8;
    audioFrame.header.byte_rate = audioFrame.header.sample_rate * 1 * 2;
    audioFrame.header.fmt_chunk_size = 16;
        audioFrame.header.bit_depth = 16;
    audioFrame.header.data_bytes = WAVSIZE * 1 * 2;
    audioFrame.header.sample_alignment = 1 * 2;

    mqttSendByteStream("hermes/audioServer/livingroom/audioFrame",
                       (const char *) &audioFrame, sizeof(audioFrame));

    return true;
  }
  return false;
}


