/*
 * wave_header.h
 *
 * Created: 30.01.2018 16:21:14
 *  Author: schiesser
 */ 


#ifndef WAVE_HEADER_H__
#define WAVE_HEADER_H__


#define PCM_BUF_SIZE						16
//#define NB_BUFFERS							4
#define AUDIO_NUM_CHANNELS					1
#define AUDIO_BITS_PER_SAMPLE				16
#define AUDIO_SAMPLING_RATE					44100
#define NUMBER_OF_CHANNELS					0x01
/* Byte rate = SampleRate * NumChannels * BitPerChannel/8
 *    br     =   44100    *      1      *       16/8     = 88'200 (0x00015888) */
#define BYTE_RATE_LL						0x88
#define BYTE_RATE_LH						0x58
#define BYTE_RATE_HL						0x01
#define BYTE_RATE_HH						0x00

/*Wave header for PCM sound file */
__ALIGN(4) int8_t wave_header[44]= {
	0x52, 0x49, 0x46, 0x46,  /*'R', 'I, 'F', 'F' */
	0x00, 0x00, 0x00, 0x00,  /* Chunk Size = subchunk1+subchunk2 size */
	0x57, 0x41, 0x56, 0x45,  /*'W', 'A, 'V', 'E' */
	0x66, 0x6d, 0x74, 0x20,  /* 'f', 'm', 't', 0x20 */
	0x10, 0x00, 0x00, 0x00,  /* Subchunk1 size - 16 for PCM */
	0x01, 0x00,              /* Audio format - PCM */
	NUMBER_OF_CHANNELS, 0x00,/* Number of channels - 1 for mono 2 for stereo*/
	0x44, 0xAC, 0x00, 0x00,  /* Sampling rate -  44100 (0xAC44)*/
	BYTE_RATE_LL, BYTE_RATE_LH, BYTE_RATE_HL, BYTE_RATE_HH,  /* Byte rate = SampleRate * NumChannels 
								* BitsPerSample/8 */
	NUMBER_OF_CHANNELS*2, 0x00, /* Block Alignment 
							  * = NumChannels * BitsPerSample/8 = 2*/
	0x10, 0x00,              /* Bits per samples = 16*/
	0x64, 0x61, 0x74, 0x61,  /*'d', 'a', 't', 'a' */
	0x00, 0x00, 0x00, 0x00,  /* subchunk2 size - total size of samples */
};

//#define AUDIO_CHUNK_SIZE					256
//#define AUDIO_BUFFER_NUMBER					4

#define WAVE_FORMAT_CHUNK_SIZE_OFFSET		04
#define WAVE_FORMAT_NUM_CHANNEL_OFFSET		22
#define WAVE_FORMAT_SAMPLE_RATE_OFFSET		24
#define WAVE_FORMAT_BYTE_RATE_OFFSET		28
#define WAVE_FORMAT_BLOCK_ALIGN_OFFSET		32
#define WAVE_FORMAT_BITS_PER_SAMPLE_OFFSET	34
#define WAVE_FORMAT_SUBCHUNK2_SIZE_OFFSET	40
#define WAVE_FORMAT_DATA_OFFSET				44


#endif /* WAVE_HEADER_H__ */
