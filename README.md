# mh-z19b_examples

## Raspberry Pi

```
cd raspberry_pi
mkdir build
cd build
cmake ..
```

## Raspberry Pico

```
cd raspberry_pico
mkdir build
cd build
cmake -D CMAKE_C_COMPILER=/usr/bin/arm-none-eabi-gcc -D CMAKE_CXX_COMPILER=/usr/bin/arm-none-eabi-g++ ..
```
