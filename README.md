# kinara
The KINARA finite state model checking and synthesis framework

To download sources:

git clone --recursive https://github.com/abhishekudupa/kinara.git

To build:

1. Go to the base kinara directory.
2. cd thirdparty/boost-local
3. make
4. cd ../z3-4.3.2
5. make
6. cd ../../
7. make eoptlto

Look in the test/mc directory for some examples on how to use the API

I know the documentation is sketchy at the moment, but I have some major refactoring planned and will update the documentation when that's done!
