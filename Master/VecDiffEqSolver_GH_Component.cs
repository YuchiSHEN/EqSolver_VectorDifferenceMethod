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

/// <summary>
/// This class will be instantiated on demand by the Script component.
/// </summary>
public class Script_Instance : GH_ScriptInstance
{
#region Utility functions
  /// <summary>Print a String to the [Out] Parameter of the Script component.</summary>
  /// <param name="text">String to print.</param>
  private void Print(string text) { __out.Add(text); }
  /// <summary>Print a formatted String to the [Out] Parameter of the Script component.</summary>
  /// <param name="format">String format.</param>
  /// <param name="args">Formatting parameters.</param>
  private void Print(string format, params object[] args) { __out.Add(string.Format(format, args)); }
  /// <summary>Print useful information about an object instance to the [Out] Parameter of the Script component. </summary>
  /// <param name="obj">Object instance to parse.</param>
  private void Reflect(object obj) { __out.Add(GH_ScriptComponentUtilities.ReflectType_CS(obj)); }
  /// <summary>Print the signatures of all the overloads of a specific method to the [Out] Parameter of the Script component. </summary>
  /// <param name="obj">Object instance to parse.</param>
  private void Reflect(object obj, string method_name) { __out.Add(GH_ScriptComponentUtilities.ReflectType_CS(obj, method_name)); }
#endregion

#region Members
  /// <summary>Gets the current Rhino document.</summary>
  private RhinoDoc RhinoDocument;
  /// <summary>Gets the Grasshopper document that owns this script.</summary>
  private GH_Document GrasshopperDocument;
  /// <summary>Gets the Grasshopper script component that owns this script.</summary>
  private IGH_Component Component; 
  /// <summary>
  /// Gets the current iteration count. The first call to RunScript() is associated with Iteration==0.
  /// Any subsequent call within the same solution will increment the Iteration count.
  /// </summary>
  private int Iteration;
#endregion

  /// <summary>
  /// This procedure contains the user code. Input parameters are provided as regular arguments, 
  /// Output parameters as ref arguments. You don't have to assign output parameters, 
  /// they will have a default value.
  /// </summary>
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

  private List<string> __err = new List<string>(); //Do not modify this list directly.
  private List<string> __out = new List<string>(); //Do not modify this list directly.
  private RhinoDoc doc = RhinoDoc.ActiveDoc;       //Legacy field.
  private IGH_ActiveObject owner;                  //Legacy field.
  private int runCount;                            //Legacy field.
  
  public override void InvokeRunScript(IGH_Component owner, object rhinoDocument, int iteration, List<object> inputs, IGH_DataAccess DA)
  {
    //Prepare for a new run...
    //1. Reset lists
    this.__out.Clear();
    this.__err.Clear();

    this.Component = owner;
    this.Iteration = iteration;
    this.GrasshopperDocument = owner.OnPingDocument();
    this.RhinoDocument = rhinoDocument as Rhino.RhinoDoc;

    this.owner = this.Component;
    this.runCount = this.Iteration;
    this. doc = this.RhinoDocument;

    //2. Assign input parameters
        List<Line> EdgeLn = null;
    if (inputs[0] != null)
    {
      EdgeLn = GH_DirtyCaster.CastToList<Line>(inputs[0]);
    }
    List<double> EdgeMag = null;
    if (inputs[1] != null)
    {
      EdgeMag = GH_DirtyCaster.CastToList<double>(inputs[1]);
    }
    List<Point3d> NodePt = null;
    if (inputs[2] != null)
    {
      NodePt = GH_DirtyCaster.CastToList<Point3d>(inputs[2]);
    }
    List<Line> ExtForceLn = null;
    if (inputs[3] != null)
    {
      ExtForceLn = GH_DirtyCaster.CastToList<Line>(inputs[3]);
    }
    List<Point3d> DefSuppNode = null;
    if (inputs[4] != null)
    {
      DefSuppNode = GH_DirtyCaster.CastToList<Point3d>(inputs[4]);
    }
    List<Point3d> FreeSuppNode = null;
    if (inputs[5] != null)
    {
      FreeSuppNode = GH_DirtyCaster.CastToList<Point3d>(inputs[5]);
    }
    int iter = default(int);
    if (inputs[6] != null)
    {
      iter = (int)(inputs[6]);
    }

    double Deform_Ratio = default(double);
    if (inputs[7] != null)
    {
      Deform_Ratio = (double)(inputs[7]);
    }

    double SubD = default(double);
    if (inputs[8] != null)
    {
      SubD = (double)(inputs[8]);
    }

    bool WeightVec = default(bool);
    if (inputs[9] != null)
    {
      WeightVec = (bool)(inputs[9]);
    }

    bool Run = default(bool);
    if (inputs[10] != null)
    {
      Run = (bool)(inputs[10]);
    }



    //3. Declare output parameters
      object Nodes = null;
  object IntForceLns = null;
  object IntForceMags = null;
  object ExtForceLns = null;


    //4. Invoke RunScript
    RunScript(EdgeLn, EdgeMag, NodePt, ExtForceLn, DefSuppNode, FreeSuppNode, iter, Deform_Ratio, SubD, WeightVec, Run, ref Nodes, ref IntForceLns, ref IntForceMags, ref ExtForceLns);
      
    try
    {
      //5. Assign output parameters to component...
            if (Nodes != null)
      {
        if (GH_Format.TreatAsCollection(Nodes))
        {
          IEnumerable __enum_Nodes = (IEnumerable)(Nodes);
          DA.SetDataList(1, __enum_Nodes);
        }
        else
        {
          if (Nodes is Grasshopper.Kernel.Data.IGH_DataTree)
          {
            //merge tree
            DA.SetDataTree(1, (Grasshopper.Kernel.Data.IGH_DataTree)(Nodes));
          }
          else
          {
            //assign direct
            DA.SetData(1, Nodes);
          }
        }
      }
      else
      {
        DA.SetData(1, null);
      }
      if (IntForceLns != null)
      {
        if (GH_Format.TreatAsCollection(IntForceLns))
        {
          IEnumerable __enum_IntForceLns = (IEnumerable)(IntForceLns);
          DA.SetDataList(2, __enum_IntForceLns);
        }
        else
        {
          if (IntForceLns is Grasshopper.Kernel.Data.IGH_DataTree)
          {
            //merge tree
            DA.SetDataTree(2, (Grasshopper.Kernel.Data.IGH_DataTree)(IntForceLns));
          }
          else
          {
            //assign direct
            DA.SetData(2, IntForceLns);
          }
        }
      }
      else
      {
        DA.SetData(2, null);
      }
      if (IntForceMags != null)
      {
        if (GH_Format.TreatAsCollection(IntForceMags))
        {
          IEnumerable __enum_IntForceMags = (IEnumerable)(IntForceMags);
          DA.SetDataList(3, __enum_IntForceMags);
        }
        else
        {
          if (IntForceMags is Grasshopper.Kernel.Data.IGH_DataTree)
          {
            //merge tree
            DA.SetDataTree(3, (Grasshopper.Kernel.Data.IGH_DataTree)(IntForceMags));
          }
          else
          {
            //assign direct
            DA.SetData(3, IntForceMags);
          }
        }
      }
      else
      {
        DA.SetData(3, null);
      }
      if (ExtForceLns != null)
      {
        if (GH_Format.TreatAsCollection(ExtForceLns))
        {
          IEnumerable __enum_ExtForceLns = (IEnumerable)(ExtForceLns);
          DA.SetDataList(4, __enum_ExtForceLns);
        }
        else
        {
          if (ExtForceLns is Grasshopper.Kernel.Data.IGH_DataTree)
          {
            //merge tree
            DA.SetDataTree(4, (Grasshopper.Kernel.Data.IGH_DataTree)(ExtForceLns));
          }
          else
          {
            //assign direct
            DA.SetData(4, ExtForceLns);
          }
        }
      }
      else
      {
        DA.SetData(4, null);
      }

    }
    catch (Exception ex)
    {
      this.__err.Add(string.Format("Script exception: {0}", ex.Message));
    }
    finally
    {
      //Add errors and messages... 
      if (owner.Params.Output.Count > 0)
      {
        if (owner.Params.Output[0] is Grasshopper.Kernel.Parameters.Param_String)
        {
          List<string> __errors_plus_messages = new List<string>();
          if (this.__err != null) { __errors_plus_messages.AddRange(this.__err); }
          if (this.__out != null) { __errors_plus_messages.AddRange(this.__out); }
          if (__errors_plus_messages.Count > 0) 
            DA.SetDataList(0, __errors_plus_messages);
        }
      }
    }
  }
}