The algorithm is splitted into two main steps:
- the key validation, allowing to check if the key is valid
- the preamble derivation, allowing to generate a two-bytes long preamble from the key

The key is considered valid if a value derived from the key is equals to zero.
This value is composed of a serie of expressions linked to the key, which are then combined together using a or bitwise operator.

- The first expression takes the 3rd byte of the key, applies a mask of value 0xec using a and bitwise operator, then is xored with value 0x20
- The second expression takes the 3rd and 4th bytes of the key, xor them together, applies a mask of value 0x3f using a and bitwise operator, then is xored with value 0x1a
- The third expression takes the 3rd, the 4th and the 5th bytes of the key, xor them together, applies a mask of value 0xd7 using a and bitwise operator, then is xored with value 0x47
- The fourth expression takes the 3rd, the 4th and the 5th and the 6th bytes of the key, xor them together, applies a mask of value 0xdb using a and bitwise operator, then is xored with value 0x11
- The fifth expression takes the 3rd, the 4th , the 5th, the 6th and the 7th bytes of the key, xor them together, applies a mask of value 0x79 using a and bitwise operator, then is xored with value 0x50
- The sixth expression takes the 3rd, the 4th , the 5th, the 6th, the 7th and the 8th bytes bytes of the key, xor them together, applies a mask of value 0xf7 using a and bitwise operator, then is xored with value 0x93
- The seventh expression takes the 2th, 3rd, the 4th , the 5th, the 6th, the 7th and the 8th bytes of the key, xor them together, applies a mask of value 0xbe using a and bitwise operator, then is xored with value 0x36
- The last expression takes every bytes of the key, xor them together, applies a mask of value 0xef using a and bitwise operator, then is xored with value 0x8f

If the key is valid, the preamble is then generated by generating two bytes derived from the key:

The least significant byte is formed by combining several expressions using a or bitwise operation:
- the first expression takes the 2th, 3rd, the 4th , the 5th, the 6th, the 7th and the 8th bytes, xor them together, and applies a mask of value 0x41 using an and bitwise operator
- the second expression takes every bytes of the key, xor them together, and applies a mask of value 0x10 using an and bitwise operator
- the third expression takes the 3rd, the 4th and the 5th bytes of the key, xor them together, and applies a mask of value 0x28 using an and bitwise operator
- the fourth expression takes the 3rd, the 4th , the 5th, the 6th and the 7th bytes of the key, xor them together, and applies a mask of value 0x86 using an and bitwise operator

the most significant byte is formed by combining several expressions using a or bitwise operation:
- the first expression takes the 3rd, the 4th , the 5th, the 6th, the 7th and the 8th bytes, xor them together, and applies a mask of value 0x08 using an and bitwise operator
- the second expression takes the 3rd and the 4th bytes of the key, xor them together, and applies a mask of value 0xc0 using an and bitwise operator
- the third expression takes the 3rd byte of the key and applies a mask of value 0x13 using an and bitwise operator
- the fourth expression takes the 3rd, the 4th, the 5th and the 6th bytes of the key, xor them together, and applies a mask of value 0x24 using an and bitwise operator
