# Vector Difference Method

This is a novel form-finding approach for the equilibrium-based design of strut-and-tie models.
<br>
__Users cam input a conceptual structural model characterized by tension and compression forces, the solver iteratively identifies the nearest equilibrium state of the form and calculates the associated forces.__
The current version supports the contrainted form-finding by adding the form constraints within the 'goal' system in K2 (Kangaroo2).
The method is implemented as a C# component for the CAD environment [McNeel Rhino/Grasshopper](https://www.rhino3d.com/) for both Windows and MacOS.<br>
You can also refer to the .NET assembly [VecDiffEqSolverBeta.dll] in C# code to run this method in customized workflow.
<br>

<div align="center">
<img src="https://github.com/YuchiSHEN/EqSolver_VectorDifferenceMethod/blob/5e0e550f401641ab598b3d6bc58817c41dee7a80/png/lattice.png" width=750>
</div>

<div align="center">
<img src="https://github.com/YuchiSHEN/EqSolver_VectorDifferenceMethod/blob/9a2b2646f5538654c21bbadc563fbf5b0313bbff/gif/Beam_Para.gif" width=750>
</div>

<br>
<br>

This method is currently developed and mantained by:
- __Yuchi Shen__ [Southeast University of Nanjing, School of Architecture](http://arch.seu.edu.cn/jz_en/main.htm)
- __Yinan Xiao__ []()
- __Pierluigi D'Acunto__ [Technical University of Munich, Professorship of Structural Design](https://www.arc.ed.tum.de/sd/structural-design/)
- __Patrick Ole Ohlbrock__ [ETH Zurich, Chair of Structural Design](https://schwartz.arch.ethz.ch/)
<br>
<br>

If you use the method, please reference the official GitHub repository: <br>
@Misc{VecDiffEqSolver, <br>
author = {Yuchi Shen, Yinan Xiao, Pierluigi D'Acunto, Patrick Ole Ohlbrock}, <br>
title = {{EqulibriumSolver_VectorDifference}}, <br>
year = {2023}, <br>
note = {Release 1.00 Beta}, <br>
url = { https://github.com/YuchiSHEN/EqSolver_VectorDifferenceMethod.git }, <br>
}
<br>
<br>
