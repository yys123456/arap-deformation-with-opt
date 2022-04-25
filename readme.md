## As rigid as possible deformation with  [niessner/Opt](https://github.com/niessner/Opt)

### prerequisite

CUDA9.2，vs2017，opengl4.5，OpenMesh

### Get it work

1. add terra/bin to system path
2. go to `terra/share/terra/tests/` and test if terra can work properly (especially those start with cuda...)
3. go to `OptBuild/build`, run build.bat, if it run successfully, `Opt.dll` and `Opt.lib` will be generated
4. add Opt.h to `include`, Opt.dll to source file path, Opt.lib to `lib`
5. go to `arap.sln`, and hit the build button

### Usage

1. S, D: toggle select and deform mode
2. W: toggle wire mode
3. A: toggle anchor points
4. mouse left/right/middle: drag mesh/(select, deform)/save mesh

### Note

1. dragging vertices too far away from their current position is not recommended
2. fitting weight and regularization weight can be changed to view different impact of them on the deformation
<!-- <img src = "https://i.imgur.com/5rRk8pQ.gif">-->

<p align="center">
  <img src="https://i.imgur.com/zjkJxL3.gif" />
   <figcaption align = "center"><b>Fitting=0.625</b></figcaption>
</p>
<p align="center">
	<img src  ="https://i.imgur.com/GtoinH3.gif">
    <figcaption align = "center"><b>Fitting=1</b></figcaption>
</p>



