## As rigid as possible deformation with  [niessner/Opt](https://github.com/niessner/Opt)

### prerequisite

CUDA9.2，vs2017，opengl4.5，OpenMesh

### Get it work

1. add terra/bin to system path
2. go to `terra/share/terra/tests/` and test if terra can work properly (especially those start with cuda...)
3. go to `OptBuild/build`, run build.bat, if it run successfully, `Opt.dll` and `Opt.lib` will be generated
4. add Opt.h to `include`, Opt.dll to source file path, Opt.lib to `lib`
5. go to `arap.sln`, and hit the build button



