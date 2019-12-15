﻿using System.Collections.Generic;
using System.Diagnostics;
using System.IO;

namespace mapf
{
    public class AdditivePDBs : IHeuristicCalculator<WorldState>
    {
        List<PDB> PDBs; 

        /// <summary>
        /// The set of agents that are not covered by this set of pattern
        /// databases. This information is not used by this class or any of its
        /// related classes, but is simply precomputed as useful information for
        /// the caller (for example, if the caller wanted to add to our
        /// heuristic estimate).
        /// </summary>
        public SortedSet<uint> excludedAgents;

        /// <summary>
        /// Determines how many additive pattern databases to build and divides
        /// the agents among them, possibly leaving some agents out.
        /// </summary>
        /// <param name="pi">the problem instance to use</param>
        /// <param name="s">The root of the search tree. This is also expected
        /// to have context parameters such as agents' goal states.</param>
        public void build(ProblemInstance pi, WorldState s)
        {
            Debug.Write("Building database...");

             // As a simple rule, we'll simply take pairs of agents starting
             // with the first two, then the second two, etc.

            PDBs = new List<PDB>();
            if (s.allAgentsState.Length > 1)
            {
                for (uint i = 0; i < s.allAgentsState.Length - 1; i += 2)
                {
                     // Make a list of agents we want to include together in the
                     // next additive pattern database. We specify agents by
                     // their index into the WorldState.allAgentsState
                     // array.

                    List<uint> agentsToConsider = new List<uint>();
                    agentsToConsider.Add(i);
                    agentsToConsider.Add(i + 1);

                     // Create a new root search node where the state only
                     // includes a subset of the agents of the original search
                     // node. This is done by passing into the state copy
                     // constructor our list of important agents.

                    WorldState tws = new WorldState(s.allAgentsState, agentsToConsider);

                     // Initialize, build, and save the new pattern database.

                    EnumeratedPDB pdb = new EnumeratedPDB();
                    pdb.Init(pi, agentsToConsider);
                    pdb.build();
                    Debug.Write(".");
                    PDBs.Add(pdb);
                }
            }

             // Create single shortest path pattern database heuristics for the
             // remaining agents if we have any left over.

            if (s.allAgentsState.Length % 2 == 1)
            {
                SumIndividualCosts pdb = new SumIndividualCosts();
                List<uint> agentsToConsider = new List<uint>(1);
                agentsToConsider.Add((uint) s.allAgentsState.Length - 1);
                pdb.Init(pi, agentsToConsider);
                pdb.build();
                PDBs.Add(pdb);
            }

             // For informational purposes, we will set the number of agents
             // that aren't included in this set of pattern databases.

            excludedAgents = new SortedSet<uint>();

            Debug.WriteLine("done.");
        }

        /// <summary>
        /// Initializes the pattern database by storing references to the
        /// problem instance and also the subset of agents that the pattern
        /// database pertains to.
        /// </summary>
        /// <param name="pi">The problem instance.</param>
        /// <param name="vAgents">The agents that the pattern database should keep track of.</param>
        public virtual void Init(ProblemInstance pi, List<uint> vAgents) {}

        /// <summary>
        /// Simply returns the sum of each of the additive pattern database 
        /// heuristic estimates on the given state.
        /// </summary>
        /// <param name="s">The state.</param>
        /// <returns>The admissible heuristic value for the additive pattern
        /// databases.</returns>
        public uint h(WorldState s)
        {
            uint nHeuristicValue = 0;
            foreach (PDB p in PDBs)
            {
                nHeuristicValue += p.h(s);
            }
            return nHeuristicValue;
        }

        public bool empty()
        {
            return PDBs.Count == 0;
        }

        /// <summary>
        /// Prints header of statistics of a single run to the given output. 
        /// </summary>
        public void OutputStatisticsHeader(TextWriter output) { }

        /// <summary>
        /// Prints statistics of a single run to the given output.
        /// </summary>
        public void OutputStatistics(TextWriter output) { }

        public int NumStatsColumns {
            get
            {
                return 0;
            }
        }

        /// <summary>
        /// Clears statistics.
        /// </summary>
        public void ClearStatistics() { }

        public void ClearAccumulatedStatistics() { }
        public void AccumulateStatistics() { }
        public void OutputAccumulatedStatistics(TextWriter output) { }

        public override string ToString()
        {
            return this.GetName();
        }

        public string GetName()
        {
            return "Additive PDB";
        }
    }
}
