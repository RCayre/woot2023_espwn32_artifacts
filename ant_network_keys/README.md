# ANT Network Keys derivation artifact

This folder contains the specification describing the validation of ANT network keys and the derivation of the preamble, and an implementation by a third person (thanks to Romain Malmain for his help on this !).

In *spec.md*, you can find the specification written after a reverse engineering process of the ANT NRF52 SoftDevice.

## Testing the implementation

To use the implementation, use :

`
$ make
`

Then you can run the binary and provide a key as an hexadecimal stream to check the validity and generate the corresponding ANT preamble:

`
$ cd build
$ ./key
`

For example, you can provide the ANT+ Network Key (45C372BDFB21A5B9), which should be valid and generate the preamble 0xc5a6:

`
$ ./key
Key: 45C372BDFB21A5B9

The provided key is correct, and the corresponding preamble is: 0xc5a6
`

You can also try the ANT-FS Network Key (C1635EF5B923A4A8), which should also be valid and generate the preamble 0xa33b:

`
$ ./key
Key: 45C372BDFB21A5B9

The provided key is correct, and the corresponding preamble is: 0xc5a6
`

Finally, if you try an incorrect key like 1122334455667788, it should indicate that the key is invalid and don't generate any preamble.

`
$ ./key
Key: 1122334455667788

Error: invalid key
`
