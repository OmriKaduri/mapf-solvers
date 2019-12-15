﻿
namespace mapf
{
    /// <summary>
    /// This Solver runs A* but from time to time tries to solve completely the expanded node
    /// using Standley's independence detection.
    /// </summary>
    class A_Star_WithRID : A_Star_WithOD
    {

        override public string GetName() { return "A*+RID"; }

        // <summary>
        // Expand a given node. This includes:
        // - Generating all possible children
        // - Inserting them to OPEN
        // - Insert the generated nodes to the hashtable of nodes, currently implmented together with the closed list.
        // </summary>
        // <param name="parent"></param>

    }
}
