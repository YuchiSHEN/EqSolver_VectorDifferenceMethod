# EqulibriumSolver_VectorDifference

This is an equilibrium solver for strut-and-tie structures, predicated on minimizing vector differences within the force polygon. By inputting a conceptual structural model characterized by tension and compression forces, the solver iteratively identifies the nearest equilibrium state of the form and calculates the associated forces. Furthermore, constraint-based form finding can be conducted by fixing points within the model.

 The method is implemented as a C# component for the CAD environment [McNeel Rhino/Grasshopper](https://www.rhino3d.com/) for both Windows and MacOS.<br>
 You can also refer to the .NET assembly [VecDiffEqSolverBeta.dll] in C# code to run this method in customized workflow.
<br>
<br>

<div align="center">
<img src="https://github.com/YuchiSHEN/EqSolver_VectorDifferenceMethod/blob/5e0e550f401641ab598b3d6bc58817c41dee7a80/png/lattice.png">
</div>

<div align="center">
<img src="https://github.com/YuchiSHEN/EqSolver_VectorDifferenceMethod/blob/9a2b2646f5538654c21bbadc563fbf5b0313bbff/gif/Beam_Para.gif">
</div>

<br>
<br>

This method is currently developed and mantained by:
- __Yuchi Shen__ [Southeast University of Nanjing, School of Architecture](http://arch.seu.edu.cn/jz_en/main.htm)
- __Pierluigi D'Acunto__ [Technical University of Munich, Professorship of Structural Design](https://www.arc.ed.tum.de/sd/structural-design/)
- __Patrick Ole Ohlbrock__ [ETH Zurich, Chair of Structural Design](https://schwartz.arch.ethz.ch/)
<br>
<br>

If you use the method, please reference the official GitHub repository: <br>
@Misc{VecDiffEqSolver, <br>
author = {Shen Yuchi, Pierluigi D'Acunto, Patrick Ole Ohlbrock}, <br>
title = {{EqulibriumSolver_VectorDifference}}, <br>
year = {2023}, <br>
note = {Release 1.00 Beta}, <br>
url = { https://github.com/YuchiSHEN/EqSolver_VectorDifferenceMethod.git }, <br>
}
<br>
<br>
