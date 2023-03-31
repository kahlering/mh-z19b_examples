# mh-z19b_examples

## Raspberry Pi

```
cd raspberry_pi
mkdir build
cd build
cmake ..
```

## Raspberry Pico
You have to install a cross compiler for Arm Cortex-M like arm-none-eabi-gcc.
Then, set the compiler path accordingly with CMAKE_C_COMPILER and CMAKE_CXX_COMPILER when executing cmake.

```
export PICO_SDK_PATH=/path/to/sdk
cd raspberry_pi_pico
mkdir build
cd build
cmake -D CMAKE_C_COMPILER=/usr/bin/arm-none-eabi-gcc -D CMAKE_CXX_COMPILER=/usr/bin/arm-none-eabi-g++ ..
make
```
This should create the file pico_example.uf2 in the build folder that can be flashed on the pico.
