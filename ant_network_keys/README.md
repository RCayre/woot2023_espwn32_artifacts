# ANT Network Keys derivation artifact

This folder contains the specification describing the validation of ANT network keys and the derivation of the preamble, and an implementation by a third person (thanks to Romain Malmain for his help on this !).

In *spec.md*, you can find the specification written after a reverse engineering process of the ANT NRF52 SoftDevice.

To use the implementation, use :

`
$ make
`

Then you can run the binary and provide a key to check the validity and generate the corresponding ANT preamble:

`
$ cd build
$ ./key
`
