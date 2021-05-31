# ML4KP ([website](https://sites.google.com/scarletmail.rutgers.edu/ml4kp/))

A library for integrating machine learning tools with state-of-the-art sampling-based kinodynamic planning algorithms.

## CURRENTLY REQUIRED DEPENDENCIES

* Eigen
* Yaml-Cpp
### _(Optional)_ Bullet Physics

1. Add the ML4KP directory to `$DIRTMP_PATH`.

2. Clone Bullet:
git clone https://github.com/aravindsiv/bullet3.git

3. Build Bullet:
```
cmake .. && make -j4 && make install
```

4. Add the Bullet directory to `$BULLET_PHYSICS_PATH`.


### _(Optional)_ LibTorch
1. Download and extract the stable build of LibTorch (CPU) to `src/prx/external`. 
2. Add the location to `$TorchDIR`.


## SETUP

Add the DirtMP directory to DIRTMP_PATH.

```
cd ~ /PATH/TO/DIRTMP/
export DIRTMP_PATH=$(pwd)
```

Add to `.bashrc` (most linux distros & older macos):

```
cd ~/PATH/TO/DIRTMP/
echo "export DIRTMP_PATH=$(pwd)" >> ~/.bashrc
```

Add `.zshrc` (macOS & some linux distros):

```
cd ~/PATH/TO/DIRTMP/
echo "export DIRTMP_PATH=$(pwd)" >> ~/.zshrc
```

## COMPILING
To build the package:

```
cd $DIRTMP_PATH
mkdir build
cd build
cmake ..
make -j4
```

When you make code changes, you only need to `make` inside the `build` directory:
```
make -j4
```


### Compilation options
By default, all options are on. To turn off any option, set it off while configuring:
I.e. to turn off Tests:
```
cmake -DBUILD_TESTS=OFF ..
make -j4
```

- `BUILD_FOR_PYTHON`: Compile python bindings
- `BUILD_FOR_BULLET`: Compile the bullet interface
- `BUILD_FOR_TORCH`:  Compile using the Torch library
- `BUILD_TESTS`: 	    Compile the unit tests. Mostly for developers of the library.

### Running unit tests
```
cmake -DBUILD_TESTS=ON .. # By default, this option is turned on
make -j4
ctest
```

## EXECUTING
To run an executable,
```
cd ../bin
./EXECUTABLE_NAME
```

## Using Python

Set pythonpath (temporal):
```
export PYTHONPATH=${DIRTMP_PATH}/lib
```

Add to `.bashrc` (most linux distros & older macos):
```
echo "export PYTHONPATH=\${DIRTMP_PATH}/lib:\${PYTHONPATH}" >> .bashrc
```

Add to `.zshrc` (macOS & some linux distros):
```
echo "export PYTHONPATH=\${DIRTMP_PATH}/lib:\${PYTHONPATH}" >> .zshrc
```

Running some examples:
```
python3.9 ${DIRTMP_PATH}/examples/acrobot/rrt.py
python3.9 ${DIRTMP_PATH}/examples/acrobot/sst.py
python3.9 ${DIRTMP_PATH}/examples/acrobot/dirt.py
```

## Issues / Bugs / Feature Requests

Please create an issue on GitHub. Be as descriptive as possible and provide a minimal working example that reproduces the issue if you can.
