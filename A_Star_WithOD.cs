﻿using System;
using System.Collections.Generic;
using System.IO;

namespace mapf
{
    /// <summary>
    /// A* implementation with Standley's operator decomposition (OD).
    /// See AAAI 2010 paper by Trevor Standley on Cooperative Pathfinding.
    /// </summary>
    public class A_Star_WithOD : A_Star
    {
        protected int expandedFullStates;
        protected int accExpandedFullStates;
        protected int generatedFullStates;
        protected int accGeneratedFullStates;

        public A_Star_WithOD(IHeuristicCalculator<WorldState> heuristic = null, bool mStar = false, bool mStarShuffle = false)
            : base(heuristic, mStar, mStarShuffle) { }

        override protected WorldState CreateSearchRoot(int minDepth = -1, int minCost = -1, MDDNode mddNode = null)
        {
            return new WorldStateWithOD(this.instance.agents, minDepth, minCost, mddNode);
        }

        protected override WorldState CreateSearchNode(WorldState from)
        {
            return new WorldStateWithOD((WorldStateWithOD)from);
        }

        public override string GetName() { return base.GetName() + "+OD"; }

        public override void Setup(ProblemInstance problemInstance, int minDepth, Run runner,
                                   int minCost = -1, int maxCost = int.MaxValue, MDD mdd = null)
        {
            base.Setup(problemInstance, minDepth, runner, minCost, maxCost, mdd);
            this.expandedFullStates = 0;
            this.generatedFullStates = 0;
        }

        protected bool alreadyExpanded;

        // Problem 59 of 3 agents and 9 obstacles is a good example of why
        // equivalence over different times is difficult to get right
        // with A*+OD (but possible).

        /// <summary>
        /// Expand a given node. This includes:
        /// - Generating all possible children
        /// - Inserting them to OPEN
        /// - Insert the generated nodes to the hashtable of nodes, currently implmented together with the closed list.
        /// </summary>
        public override void Expand(WorldState node)
        {
            if (((WorldStateWithOD)node).agentTurn == 0)
                expandedFullStates++;
            this.alreadyExpanded = false;
            base.Expand(node);
        }

        protected override List<WorldState> ExpandOneAgent(List<WorldState> intermediateNodes, int agentIndex)
        {
            if (this.alreadyExpanded == true)  // Necessary because after expansion, the generated nodes have an incremented agentTurn that once again equals agentIndex
                                               // and because it's possible that a node may have valid children that aren't already in the closed list
                return intermediateNodes; // Do nothing to this agent

            WorldStateWithOD parent = (WorldStateWithOD)intermediateNodes[0];

            if (agentIndex < parent.agentTurn)
                return intermediateNodes; // Do nothing to this agent

            var generated = base.ExpandOneAgent(intermediateNodes, agentIndex);

            int childAgentTurn = ((parent.agentTurn + 1) % (this.instance.agents.Length));
            foreach (var node in generated)
            {
                WorldStateWithOD childNode = (WorldStateWithOD)node;

                childNode.agentTurn = childAgentTurn;

                // Makespan increases only if this is the move of the first agent. This makes sure that under a makespan
                // cost function, partial nodes have a correct cost and can even serve as goal nodes.
                if (parent.agentTurn != 0)
                    childNode.makespan--; // Cancel the increment in base
            }

            this.alreadyExpanded = true;
            return generated;
        }

        protected override bool ProcessGeneratedNode(WorldState currentNode)
        {
            bool ret = base.ProcessGeneratedNode(currentNode);
            var node = (WorldStateWithOD)currentNode;
            if (node.agentTurn == 0)
                this.generatedFullStates++;
            return ret;
        }

        public override void OutputStatisticsHeader(TextWriter output)
        {
            base.OutputStatisticsHeader(output);

            output.Write(this.ToString() + " Expanded Full States");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Generated Full States");
            output.Write(Run.RESULTS_DELIMITER);
        }

        public override void OutputStatistics(TextWriter output)
        {
            base.OutputStatistics(output);

            Console.WriteLine("Total Expanded Full States: {0}", this.expandedFullStates);
            Console.WriteLine("Total Generated Full States: {0}", this.generatedFullStates);

            output.Write(this.expandedFullStates + Run.RESULTS_DELIMITER);
            output.Write(this.generatedFullStates + Run.RESULTS_DELIMITER);
        }

        public override int NumStatsColumns
        {
            get
            {
                return 2 + base.NumStatsColumns;
            }
        }

        public override void ClearAccumulatedStatistics()
        {
            base.ClearAccumulatedStatistics();

            this.accExpandedFullStates = 0;
            this.accGeneratedFullStates = 0;
        }

        public override void AccumulateStatistics()
        {
            base.AccumulateStatistics();

            this.accExpandedFullStates += this.expandedFullStates;
            this.accGeneratedFullStates += this.generatedFullStates;
        }

        public override void OutputAccumulatedStatistics(TextWriter output)
        {
            base.OutputAccumulatedStatistics(output);

            Console.WriteLine("{0} Average Expanded Full States (Low-Level): {1}", this, this.accExpandedFullStates);
            Console.WriteLine("{0} Average Generated Full States (Low-Level): {1}", this, this.accGeneratedFullStates);

            output.Write(this.accExpandedFullStates + Run.RESULTS_DELIMITER);
            output.Write(this.accGeneratedFullStates + Run.RESULTS_DELIMITER);
        }
    }
}
