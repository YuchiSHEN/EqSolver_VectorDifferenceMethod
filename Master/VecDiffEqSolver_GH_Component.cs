using System;
using System.Collections;
using System.Collections.Generic;

using Rhino;
using Rhino.Geometry;

using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;

using System.Linq;
using System.Threading.Tasks;
using VecDiffSolver;

  private void RunScript(List<Line> EdgeLn, List<double> EdgeMag, List<Point3d> NodePt, List<Line> ExtForceLn, List<Point3d> DefSuppNode, List<Point3d> FreeSuppNode, int iter, double Deform_Ratio, double SubD, bool WeightVec, bool Run, ref object Nodes, ref object IntForceLns, ref object IntForceMags, ref object ExtForceLns)
  {
        ///
    ///This is a vector-difference-based equlibrium solver, developed by:
    ///[Yuchi Shen]: Southeast University of Nanjing, School of Architecture
    ///[Pierluigi D'Acunto]: Technical University of Munich, Professorship of Structural Design
    ///<Beta version>
    ///Please make sure the assembly <VecDiffEqSolverBeta.dll> is well attached in the component.
    ///

    #region 0_Global Settings

    ///0.0 Inputs_data
    //ratio： is a form force deforming value ranged from 0~1, where 0.5 means form and force changes in the same weight to get eq;
    //WeightVectorDiff:If true, the program will weight the various difference between the force vectors.This may make the tension-compression combined structure solution better.
    //ImposeEquilibrium:If true, the solution will be imposed to a global equilirbium eventhough the optimization have not get converged. In that case, fake load may appear in the nodes of the structure.
    //tol: The tolerance for the comparing nodes in rhino in a minimal distance.If the modelling accuracy should be larger, then lower the value.
    //threshold: The main loop will stop while the convergence under the threshold.
    //subdivison:The transforming of the form diagram can be subdivided into segments to impove the accuracy of the solving.

    bool WeightVectorDiff = WeightVec;
    bool ImposeEquilibrium = true;
    double tol = 0.0001;
    double threshold = 0.0001;
    double subdivison = SubD;
    double scale = 1.0;

    #endregion

    if(Run)
    {
      #region 1_Data Construction
      //AllSuppIndex:All the support nodes that will not move in the form-finding process; AllSuppIndex contians OptSuppIndex;
      //OptSuppIndex:Certain supports that can be optimized loads in the form-finding process;
      //The difference set of above two are the supports predefined, the same with the loads;

      List<int> AllSuppIndex;
      List<int> OptSuppIndex;
      VecDiffSolver.Data.SupportMatch(NodePt, FreeSuppNode, DefSuppNode, tol, out AllSuppIndex, out OptSuppIndex);

      //LoadOnVertices: a List of Lists( with load vectors applying on the same index vertex)
      List<List<Vector3d>> LoadOnVertices;
      VecDiffSolver.Data.LoadMatch(NodePt, ExtForceLn, tol, out LoadOnVertices);

      //1.1_Construct the matrix of forces in the structure;
      double[,] Matr_force;
      VecDiffSolver.Data.ConstructForceMatrix(NodePt, EdgeLn, EdgeMag, tol, out Matr_force);
      #endregion


      #region 2_MainLoop
      //2 MainLoop
      List<Point3d> Vns = new List<Point3d>(NodePt);
      int Iteration = 0;
      double convergence = double.MaxValue;
      List<double> ratioList = NodePt.Select(p => Deform_Ratio).ToList();

      while(Iteration < iter && convergence >= threshold)
      {
        Iteration++;

        Vector3d[] AVsums;
        if(WeightVectorDiff)
        {
          //Calculate weighted Vertex Force Sum
          Vector3d[,] VecDiffMatrix;
          VecDiffSolver.VecDiff.Weight_VectorDiffOnNodes(Vns, OptSuppIndex, Matr_force, LoadOnVertices, subdivison, tol, out VecDiffMatrix, out AVsums);

          //Update Vertices
          List<Point3d> N_Vertices;
          VecDiffSolver.VecDiff.UpdateVertices(AllSuppIndex, Vns, AVsums, ratioList, Matr_force, LoadOnVertices, tol, out N_Vertices);

          //Update weighted Force Matrix
          VecDiffSolver.VecDiff.Weight_UpdateMatr_force(ref Matr_force, VecDiffMatrix, Vns, N_Vertices, Deform_Ratio, tol);
          Vns = N_Vertices;
        }
        else
        {
          //Calculate Vertex Force Sum
          VecDiffSolver.VecDiff.VectorDiffOnNodes(Vns, OptSuppIndex, Matr_force, LoadOnVertices, subdivison, tol, out AVsums);

          //Update Vertices
          List<Point3d> N_Vertices;
          VecDiffSolver.VecDiff.UpdateVertices(AllSuppIndex, Vns, AVsums, ratioList, Matr_force, LoadOnVertices, tol, out N_Vertices);

          //Update Force Matrix
          VecDiffSolver.VecDiff.UpdateMatr_force(ref Matr_force, AVsums, Vns, N_Vertices, Deform_Ratio, tol);
          Vns = N_Vertices;
        }

        //Check convergence
        convergence = 0.0;
        foreach(Vector3d vec in AVsums)
        {
          convergence += vec.Length;
        }
        if(Iteration == iter || convergence < threshold)
        {
          report = VecDiffSolver.Debug.Report(AVsums, Deform_Ratio, Iteration);
        }
      }

      #endregion


      #region 3_GetSolution
      //Get the solved equilibrium information;
      VecDiffSolver.IO.Solution(Vns, Matr_force, LoadOnVertices, OptSuppIndex, AllSuppIndex, ImposeEquilibrium, tol, scale, out Internal_Ln, out Internal_Force, out External_Ln);
      StrNodes = Vns;
      #endregion
    }

    //Outputs in GH Component
    Nodes = StrNodes;
    IntForceLns = Internal_Ln;
    IntForceMags = Internal_Force;
    ExtForceLns = External_Ln;
    Print(report);
  }

  // <Custom additional code> 
    List<Point3d>StrNodes;
  List <Line> Internal_Ln;
  List<double> Internal_Force;
  List<Line> External_Ln;
  string report;
  // </Custom additional code> 
