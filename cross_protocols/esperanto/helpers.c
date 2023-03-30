#include "helpers.h"

/* MSK to OPQSK association table */
uint8_t SYMBOL_TO_CHIP_MAPPING[16][4] = {
		{0x03,0xf7,0x3a,0x1b},
		{0x39,0x70,0xaf,0x33},
		{0x9a,0x03,0xf7,0x3a},
		{0xb2,0x39,0x70,0x2f},
		{0x3b,0x9b,0x03,0x77},
		{0xae,0xb3,0x39,0x70},
		{0xf7,0x3a,0x9b,0x03},
		{0x70,0xaf,0xb3,0x39},
		{0xfc,0x08,0xc5,0x64},
		{0xc6,0x8f,0x50,0x4c},
		{0x65,0xfc,0x08,0x45},
		{0x4d,0xc6,0x8f,0x50},
		{0xc4,0x64,0xfc,0x08},
		{0x51,0x4c,0xc6,0x0f},
		{0x08,0xc5,0x64,0x7c},
		{0x8f,0x50,0x4c,0x46},
};

uint32_t switch_endianness(uint32_t val) {
	/* Switch endianness of a uint32_t value. */
   uint32_t ret = ((val & 0x000000FF) << 24) |
                  ((val & 0x0000FF00) << 8) |
                  ((val & 0x00FF0000) >> 8) |
                  ((val & 0xFF000000) >> 24);
   return ret;
}

uint8_t swap(uint8_t val) {
		/* Swap a byte. */
   val = (val & 0xF0) >> 4 | (val & 0x0F) << 4;
   val = (val & 0xCC) >> 2 | (val & 0x33) << 2;
   val = (val & 0xAA) >> 1 | (val & 0x55) << 1;
   return val;
}

void dewhiten_ble(uint8_t *data, int len, int channel) {
		/* Dewhiten a BLE packet according to a specific channel. */
    int i,j;
    uint8_t c;
    uint8_t lfsr = swap(channel) | 2;

    for (i=0; i<len; i++)
    {
        c = swap(data[i]);
        for (j=7; j>=0; j--)
        {
            if (lfsr & 0x80)
            {
                lfsr ^= 0x11;
                c ^= (1<<j);
            }
            lfsr <<= 1;
        }
        data[i] = swap(c);
    }
}


unsigned short crc16(uint8_t *ptr, int count, int init) {
		/* Perform a CRC computation with a given data and init value (polynomial: 0x1021). */
   int  crc;
   uint8_t i;
   crc = init;
   while (--count >= 0)
   {
      crc = crc ^ (int) *ptr++ << 8;
      i = 8;
      do
      {
         if (crc & 0x8000)
            crc = crc << 1 ^ 0x1021;
         else
            crc = crc << 1;
      } while(--i);
   }
   return (crc);
}

int hamming(uint8_t *demod_buffer,uint8_t *pattern) {
	/* Compute hamming distance. */
	int count = 0;
	for (int i=0;i<4;i++) {
		for (int j=0;j<8;j++) {
			if (((pattern[i] >> (7-j)) & 0x01) != (((demod_buffer[i] & (i==0 && j==0 ? 0x7F : 0xFF)) >> (7-j)) & 0x01)) {
				count++;
			}
		}
	}
	return count;
}


void shift_buffer(uint8_t *demod_buffer,int size) {
	/* Shift a buffer by 1 bit to the left. */
	for (int i=0;i<size;i++) {
		if (i != 0) {
			demod_buffer[i-1]=((demod_buffer[i] & 0x80) >> 7) | demod_buffer[i-1];
		}
		demod_buffer[i] = demod_buffer[i] << 1;
	}
}


void msk_to_oqsk(uint8_t *oqpsk_buffer, uint32_t *oqpsk_buffer_length, uint8_t * msk_buffer) {
	/* Convert a MSK sequence to an equivalent OQPSK sequence. */
	for (int i=0;i<50;i++) oqpsk_buffer[i] = 0;
	// index of the current Dot15d4 symbol
	uint32_t index = 0;
	// indicator of current 4 bits position (1 = 4 MSB, 0 = 4 LSB)
	uint32_t part = 1;
	// indicator of start frame delimiter
	uint32_t sfd = 0;
	// Hamming distance
	uint32_t hamming_dist = 0;
	// Thresold Hamming distance
	uint32_t hamming_thresold = 8;

	// Align the buffer with the SFD
	oqpsk_buffer[0] |= (0x0F & 0x07);
	hamming_dist = 32;
	while (hamming_dist > hamming_thresold) {
		hamming_dist = hamming(msk_buffer,SYMBOL_TO_CHIP_MAPPING[0]);
		if (hamming_dist > hamming_thresold) {
			shift_buffer(oqpsk_buffer,255);
		}
	}

	hamming_dist = 0;
	while (hamming_dist <= hamming_thresold) {
		int symbol = -1;
		// Compute the hamming distance for every Dot15d4 symbol
		for (int i=0;i<16;i++) {
			hamming_dist = hamming(msk_buffer,SYMBOL_TO_CHIP_MAPPING[i]);
			if (hamming_dist <= hamming_thresold) {
				symbol = i;
				break;
			}
		}

		// If a Dot15d4 symbol has been found ...
		if (symbol != -1) {
			// If it matches the SFD next symbol, start the frame decoding
			if (sfd == 0 && symbol == 10) {
				sfd = 1;
			}

			// If we are in the frame decoding state ...
			if (sfd == 1) {
				// Fill the output buffer with the selected symbol
				oqpsk_buffer[index] |= (symbol & 0x0F) << 4*part;

				// Select the next 4 bits free space in the output buffer
				part = part == 1 ? 0 : 1;
				if (part == 0) index++;
			}
			// Shift the buffer (31 bits shift)
			for (int i=0;i<32;i++) shift_buffer(msk_buffer,255);
		}
	}

	// Export of the effective length of the Dot15d4 frame
	*oqpsk_buffer_length = index;
}
void oqpsk_to_msk(uint8_t *msk_buffer, size_t* msk_size, uint8_t *oqpsk_buffer, size_t oqpsk_buffer_size) {
		/* Convert an OQPSK sequence to an equivalent MSK sequence. */
  	for (int i=0;i<oqpsk_buffer_size;i++) {
  		uint8_t msb = oqpsk_buffer[i] >> 4;
  		uint8_t lsb = oqpsk_buffer[i] & 0x0F;

  		msk_buffer[i*8+0] = (SYMBOL_TO_CHIP_MAPPING[lsb][0]);
  		msk_buffer[i*8+1] = (SYMBOL_TO_CHIP_MAPPING[lsb][1]);
  		msk_buffer[i*8+2] = (SYMBOL_TO_CHIP_MAPPING[lsb][2]);
  		msk_buffer[i*8+3] = (SYMBOL_TO_CHIP_MAPPING[lsb][3]);

  		msk_buffer[i*8+4] = (SYMBOL_TO_CHIP_MAPPING[msb][0]);
  		msk_buffer[i*8+5] = (SYMBOL_TO_CHIP_MAPPING[msb][1]);
  		msk_buffer[i*8+6] = (SYMBOL_TO_CHIP_MAPPING[msb][2]);
  		msk_buffer[i*8+7] = (SYMBOL_TO_CHIP_MAPPING[msb][3]);
  	}
    *msk_size = ((oqpsk_buffer_size - 1) * 8 + 7) + 1;
}


uint16_t update_fcs_dot15d4(uint8_t byte, uint16_t fcs) {
	/* Helper to compute FCS of 802.15.4 packets. */
	uint16_t q = (fcs ^ byte) & 15;
	fcs = (fcs / 16) ^ (q * 4225);
	q = (fcs ^ (byte / 16)) & 15;
	fcs = (fcs / 16) ^ (q * 4225);
	return fcs;
}

uint16_t calculate_fcs_dot15d4(uint8_t *data, int size) {
	/* Compute the FCS of a 802.15.4 packet. */
	uint16_t fcs = 0;
	for (int i=2; i<size-2;i++) {
		fcs = update_fcs_dot15d4(data[i],fcs);
	}
	return fcs;
}

bool check_fcs_dot15d4(uint8_t *data,int size) {
	/* Checks if the FCS of a 802.15.4 packet is valid. */
	uint16_t fcs = calculate_fcs_dot15d4(data,size);
	return ((fcs & 0xFF) == data[size-2]) && (((fcs >> 8) & 0xFF) == data[size-1]);
}
