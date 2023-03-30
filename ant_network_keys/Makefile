all: key

key: build | key.c
	gcc key.c -Wall -Wextra -o build/key

build:
	mkdir -p build

clean:
	rm -rf build