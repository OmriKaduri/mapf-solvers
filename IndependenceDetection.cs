using System;
using System.Linq;
using System.Collections.Generic;
using System.IO;
using System.Diagnostics;
using SharpLearning.XGBoost.Learners;
using SharpLearning.XGBoost.Models;

namespace mapf
{
    class IndependenceDetection : ISolver
    {
        // The key of the illegal moves table in the ProblemInstance (used in ImprovedID())
        public static string ILLEGAL_MOVES_KEY = "ID-reserved";
        // The key of the maximal solution cost of the agent group in the ProblemInstance (used in ImprovedID())
        public static string MAXIMUM_COST_KEY = "ID-max-cost";
        // The key of the conflict avoidance table
        public static string CONFLICT_AVOIDANCE = "ID-ConflictAvoidance";

        protected LinkedList<IndependenceDetectionAgentsGroup> allGroups;
        protected ProblemInstance instance;
        protected int expanded;
        protected int generated;
        protected int accExpanded;
        protected int accGenerated;
        public int totalCost;
        protected Run runner;

        public Boolean withSelection;

        /// <summary>
        /// The complete plan for all the agents that was found.
        /// </summary>
        public Plan plan;
        
        protected int maxGroupSize;
        protected int minGroupSize;
        protected int accMaxGroupSize;
        protected int accMinGroupSize;
        public ISolver groupSolver;
        public ISolver singleAgentSolver;
        private ISet<IndependenceDetectionConflict> allConflicts;
        private int solutionDepth;
        private Dictionary<TimedMove, List<int>> conflictAvoidance;
        private int maxSolutionCostFound;

        public IndependenceDetection(ISolver singleAgentSolver, ISolver groupSolver, Boolean withSelection = false)
        {
            this.singleAgentSolver = singleAgentSolver;
            this.groupSolver = groupSolver;
            this.withSelection = withSelection;

        }

        public void Clear()
        {
            this.allGroups.Clear();
            this.groupSolver.Clear();
            this.allConflicts.Clear();
            this.conflictAvoidance.Clear();
        }

        public void Setup(ProblemInstance instance, Run runner)
        {
            this.instance = instance;
            this.runner = runner;
            this.totalCost = 0;
            this.ClearStatistics();
            this.conflictAvoidance = new Dictionary<TimedMove, List<int>>();
            this.allConflicts = new HashSet<IndependenceDetectionConflict>();
            this.allGroups = new LinkedList<IndependenceDetectionAgentsGroup>();
            // Initialize the agent group collection with a group for every agent
            foreach (AgentState agentStartState in instance.agents)
                this.allGroups.AddLast(
                    new IndependenceDetectionAgentsGroup(
                        this.instance, new AgentState[1] { agentStartState },
                        this.singleAgentSolver, this.groupSolver));
        }

        public virtual String GetName() {
            if (this.withSelection)
            {
                return groupSolver.GetName() + "+ID+AS";
            }
            else
            {
                return groupSolver.GetName() + "+ID";
            }
        }

        /// <summary>
        /// Calculate the full plan for all the agents that has been found by the algorithm
        /// </summary>
        public Plan CalculateJointPlan() 
        {
            var singlePlans = new SinglePlan[this.instance.GetNumOfAgents()];
            foreach (var group in this.allGroups)
            {
                var groupPlan = group.GetPlan();
                int i = 0;
                foreach (var agentState in group.allAgentsState)
                {
                    singlePlans[agentState.agent.agentNum] = new SinglePlan(groupPlan, i, agentState.agent.agentNum);
                    i++;
                }
                    
            }
            return new Plan(singlePlans);
        }

        public Plan GetPlan()
        {
            return this.plan;
        }

        public int GetSolutionCost() { return this.totalCost; }

        public virtual void OutputStatisticsHeader(TextWriter output)
        {
            // TODO: Use the solver's statistics, as done in CBS.
            output.Write(this.ToString() + " Expanded");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Generated");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Max Group Size");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Min Group Size");
            output.Write(Run.RESULTS_DELIMITER);
        }

        /// <summary>
        /// Prints statistics of a single run to the given output. 
        /// </summary>
        public void OutputStatistics(TextWriter output)//, BsonDocument row)
        {
            Console.WriteLine("Total Expanded Nodes: {0}", this.expanded);
            Console.WriteLine("Total Generated Nodes: {0}", this.generated);

            output.Write(this.expanded + Run.RESULTS_DELIMITER);
            output.Write(this.generated + Run.RESULTS_DELIMITER);

            // Compute and output the maximum group size
            this.maxGroupSize = 0;
            this.solutionDepth = 0;
            this.minGroupSize = this.instance.agents.Length;
            foreach (var group in this.allGroups)
            {
                this.solutionDepth += group.depthOfSolution;
                if (group.allAgentsState.Length > this.maxGroupSize)
                    this.maxGroupSize = group.allAgentsState.Length;
                if (group.allAgentsState.Length < this.minGroupSize)
                    this.minGroupSize = group.allAgentsState.Length;
            }

            Console.WriteLine("Max Group: {0}", this.maxGroupSize);
            Console.WriteLine("Min Group: {0}", this.minGroupSize);

            output.Write(this.maxGroupSize + Run.RESULTS_DELIMITER);
            output.Write(this.minGroupSize + Run.RESULTS_DELIMITER);
        }

        public int NumStatsColumns
        {
            get
            {
                return 4;
            }
        }

        public void ClearStatistics()
        {
            this.expanded = 0;
            this.generated = 0;
            this.maxGroupSize = 1;
            this.minGroupSize = instance.agents.Length;
        }

        public void ClearAccumulatedStatistics()
        {
            this.accExpanded = 0;
            this.accGenerated = 0;
            this.accMaxGroupSize = 1;
            this.accMinGroupSize = this.instance.agents.Length;
        }

        public void AccumulateStatistics()
        {
            this.accExpanded += this.expanded;
            this.accGenerated += this.generated;
            this.accMaxGroupSize = Math.Max(this.accMaxGroupSize, this.maxGroupSize);
            this.accMinGroupSize = Math.Min(this.accMinGroupSize, this.minGroupSize);
        }

        public void OutputAccumulatedStatistics(TextWriter output)
        {
            Console.WriteLine("{0} Accumulated Expanded Nodes (Low-Level): {1}", this, this.accExpanded);
            Console.WriteLine("{0} Accumulated Generated Nodes (Low-Level): {1}", this, this.accGenerated);

            output.Write(this.accExpanded + Run.RESULTS_DELIMITER);
            output.Write(this.accGenerated + Run.RESULTS_DELIMITER);

            Console.WriteLine("{0} Accumulated Max Group (Low-Level): {1}", this, this.accMaxGroupSize);
            Console.WriteLine("{0} Accumulated Min Group (Low-Level): {1}", this, this.accMinGroupSize);

            output.Write(this.accMaxGroupSize + Run.RESULTS_DELIMITER);
            output.Write(this.accMinGroupSize + Run.RESULTS_DELIMITER);
        }

        /// <summary>
        /// Also calculates min group size and max solution depth on the way.
        /// FIXME: Code dup!
        /// </summary>
        /// <returns></returns>
        public int GetMaxGroupSize()
        {
            this.solutionDepth = 0;
            this.maxGroupSize = 0;
            this.minGroupSize = int.MaxValue;
            foreach (var group in this.allGroups)
            {
                this.solutionDepth += group.depthOfSolution;
                if (group.allAgentsState.Length > this.maxGroupSize)
                    this.maxGroupSize = group.allAgentsState.Length;
                if (group.allAgentsState.Length < this.minGroupSize)
                    this.minGroupSize = group.allAgentsState.Length;
            }
            return this.maxGroupSize;
        }
        /// <summary>
        /// Simulates the execution of the plans found for the different groups. 
        /// If there are conflicting plans - return the conflicting groups.
        /// </summary>
        /// <returns>A conflict object with data about the found conflict, or null if no conflict exists</returns>
        public IndependenceDetectionConflict FindConflictingGroups()
        {
            if (this.allGroups.Count == 1)
                return null;
            
            // Find the longest plan among all the groups
            int maxPlanSize = this.allGroups.Select(group => group.GetPlan().GetSize()).Max();

            // Check in every time step that the plans do not collide
            for(int time = 1 ; time < maxPlanSize ; time++) // Assuming no conflicts exist in time zero.
            {
                int i1 = -1;
                // Check all pairs of groups for a conflict at the given time step
                foreach (var group1 in this.allGroups)
                {
                    i1++;
                    Plan group1Plan = group1.GetPlan();
                    int i2 = -1;
                    foreach (var group2 in this.allGroups)
                    {
                        i2++;
                        Plan group2Plan = group2.GetPlan();
                        if (i1 < i2 &&
                            group1Plan.IsColliding(time, group2Plan))
                            return new IndependenceDetectionConflict(group1, group2, time);
                    }
                }
            }
            return null;
        }

        /// <summary>
        /// Search for an optimal solution using the Simple Independence Detection algorithm from Trevor Standley's paper.
        /// </summary>
        /// <param name="runner"></param>
        /// <returns></returns>
        public bool SimpleID(Run runner)
        {
            while (true)
            {
                IndependenceDetectionConflict conflict = FindConflictingGroups();
                // If there are no conflicts - can finish the run
                if (conflict == null)
                    break;
                allGroups.Remove(conflict.group1);
                allGroups.Remove(conflict.group2);
                IndependenceDetectionAgentsGroup compositeGroup = this.JoinGroups(conflict);
                
                // Solve composite group with A*
                bool solved = compositeGroup.Solve(runner);
                if (solved == false)
                {
                    this.totalCost = compositeGroup.solutionCost;
                    return false;
                }

                allGroups.AddFirst(compositeGroup);
            }
            return true;
        }

        public bool debug = false;

        /// <summary>
        /// Search for an optimal solution using the Independence Detection algorithm in Standley's paper,
        /// which utilises a CAT.
        /// </summary>
        /// <param name="runner"></param>
        /// <returns></returns>
        public bool ImprovedID(Run runner)
        {
            while (true)
            {
                IndependenceDetectionConflict conflict = FindConflictingGroups();
                // If there are no conflicts - can return the current plan
                if (conflict == null)
                    break;
                
                if (this.allConflicts.Contains(conflict) == false)  // We haven't already tried to resolve this conflict
                                                                    // without merging the groups 
                {
                    // Try to solve the current conflict by re-planning one of the groups
                    if (this.debug)
                    {
                        Debug.WriteLine($"Trying to find an alternative path that avoids {conflict}");
                    }
                    
                    // Prevent trying to resolve this conflict this way again
                    this.allConflicts.Add(conflict);

                    // Add the plan of group2 to the illegal moves table and re-plan group1 with equal cost
                    if ((conflict.time < conflict.group1.GetPlan().GetSize() - 1) ||
                        (conflict.group1.allAgentsState.Length > 1)) // Otherwise the conflict is while a single agent
                                                                     // is at its goal, no chance of an alternate path
                                                                     // with the same cost that avoids the conflict
                    {
                        if (this.debug)
                        {
                            Debug.WriteLine($"Trying to find an alternative path that avoids a conflict for group 1.");
                            Debug.WriteLine($"Old plan:");
                            conflict.group1.GetPlan().PrintPlan();
                        }
                        conflict.group1.removeGroupFromCAT(conflictAvoidance);
                        bool resolved = conflict.group1.ReplanUnderConstraints(conflict.group2.GetPlan(), runner, withSelection);
                        conflict.group1.addGroupToCAT(conflictAvoidance, maxSolutionCostFound);
                        if (resolved == true)
                        {
                            if (this.debug)
                            {
                                Debug.WriteLine($"Found an alternative path that avoids a conflict for group 1:");
                                conflict.group1.GetPlan().PrintPlan();
                            }
                            continue;
                        }

                        if (this.debug)
                        {
                            Debug.WriteLine($"Couldn't find an alternative path that avoids a conflict for group 1");
                        }
                    }
                    
                    // Add the plan of group1 to the illegal moves table and re-plan group2 with equal cost
                    if ((conflict.time < conflict.group2.GetPlan().GetSize() - 1) ||
                        (conflict.group2.allAgentsState.Length > 1))
                    {
                        if (this.debug)
                        {
                            Debug.WriteLine($"Trying to find an alternative path that avoids a conflict for group 2");
                            Debug.WriteLine($"Old plan:");
                            conflict.group2.GetPlan().PrintPlan();
                        }
                        conflict.group2.removeGroupFromCAT(conflictAvoidance);
                        bool resolved = conflict.group2.ReplanUnderConstraints(conflict.group1.GetPlan(), runner, withSelection);
                        conflict.group2.addGroupToCAT(conflictAvoidance, maxSolutionCostFound);
                        if (resolved == true)
                        {
                            if (this.debug)
                            {
                                Debug.WriteLine($"Found an alternative path that avoids a conflict for group 2:");
                                conflict.group2.GetPlan().PrintPlan();
                            }
                            continue;
                        }
                        if (this.debug)
                        {
                            Debug.WriteLine($"Couldn't find an alternative path that avoids a conflict for group 2");
                        }
                    }
                }

                // Groups are conflicting - need to join them to a single group
                allGroups.Remove(conflict.group1);
                allGroups.Remove(conflict.group2);
                // Remove both groups from avoidance table
                conflict.group1.removeGroupFromCAT(conflictAvoidance);
                conflict.group2.removeGroupFromCAT(conflictAvoidance);
                if (this.debug)
                {
                    Debug.WriteLine($"Merging the agent groups that participate in {conflict}. " +
                                      $"Group1 plan before the merge: ");
                    conflict.group1.GetPlan().PrintPlan();
                    Debug.WriteLine($"Group2 plan before the merge: ");
                    conflict.group2.GetPlan().PrintPlan();
                }

                IndependenceDetectionAgentsGroup compositeGroup = this.JoinGroups(conflict);

                compositeGroup.instance.parameters[CONFLICT_AVOIDANCE] = conflictAvoidance;

                // Solve composite group with the underlying group solver
                bool solved;
                if (this.withSelection)
                {
                    solved = compositeGroup.SolveWithSelection(runner);
                }
                else
                {
                    solved = compositeGroup.Solve(runner);
                }

                if (compositeGroup.solutionCost > maxSolutionCostFound)
                    maxSolutionCostFound = compositeGroup.solutionCost;

                // Add the new group to conflict avoidance table
                compositeGroup.addGroupToCAT(conflictAvoidance, maxSolutionCostFound);
                allGroups.AddFirst(compositeGroup);

                this.expanded += compositeGroup.expanded;
                this.generated += compositeGroup.generated;

                if (compositeGroup.allAgentsState.Length > this.maxGroupSize)
                    this.maxGroupSize = compositeGroup.allAgentsState.Length;

                if (solved == false)
                {
                    this.totalCost = Constants.NO_SOLUTION_COST;
                    return false;
                }
            }
            return true;
        }

        /// <summary>
        /// Join the conflicting groups into a single group
        /// </summary>
        /// <param name="conflict">An object that describes the conflict</param>
        /// <returns>The composite group of agents</returns>
        protected virtual IndependenceDetectionAgentsGroup JoinGroups(IndependenceDetectionConflict conflict)
        {
            return conflict.group1.Join(conflict.group2);
        }

        /// <summary>
        /// Run the A* algorithm with Standley's ID and OD improvements.
        /// </summary>
        /// <returns>true if optimal solution has been found</returns>
        public bool Solve()
        {
            bool solved;
            // Solve the single agent problems independently
            this.maxSolutionCostFound = 0;

            foreach (var group in this.allGroups)
            {
                group.instance.parameters[CONFLICT_AVOIDANCE] = this.conflictAvoidance;
                solved = group.Solve(runner);

                // Check if max time has been exceeded or search failed for another reason
                if (solved == false)
                {
                    this.totalCost = group.solutionCost; // Should be some error code from Constants.
                    this.Clear();
                    return false;
                }

                if (group.solutionCost > this.maxSolutionCostFound)
                    this.maxSolutionCostFound = group.solutionCost;
                // Add group to conflict avoidance table
                group.addGroupToCAT(this.conflictAvoidance, this.maxSolutionCostFound);

                this.expanded += group.expanded;
                this.generated += group.generated;
            }

            //solved = this.SimpleID(runner);
            solved = this.ImprovedID(runner);
            // Record found solution
            if (solved == true)
            {
                // Store solution details
                this.totalCost = this.allGroups.Select(group => group.solutionCost).Sum();
                this.plan = this.CalculateJointPlan();
            }
            else
            {
                this.plan = null;
            }

            // TODO: Add a statistic for the number of groups
            return solved;
        }

        public override string ToString()
        {
            return GetName();
        }

        private void print()
        {
            Console.WriteLine("Expanded - " + expanded);
            Console.WriteLine("Generated - " + generated);
            Console.WriteLine("Total cost - " + totalCost);
        }

        public int GetExpanded() { return this.expanded; }
        public int GetGenerated() { return this.generated; }
        public int GetSolutionDepth() { return solutionDepth; }
        public long GetMemoryUsed() { return Process.GetCurrentProcess().VirtualMemorySize64; }
    }


    /// <summary>
    /// This class represents a group of agents that need to be solved together.
    /// </summary>
    class IndependenceDetectionAgentsGroup
    {
        public AgentState[] allAgentsState;
        public int solutionCost;
        public ProblemInstance instance;
        public int expanded;
        public int generated;
        public int depthOfSolution;

        private ISolver singleAgentSolver;
        private ISolver groupSolver;
        private Plan plan;

        ClassificationXGBoostModel selectionModel;
        private SumIndividualCosts simple;
        private EPEA_Star epea;
        private A_Star astar;
        private CBS macbs;
        private CostTreeSearchSolverOldMatching icts;
        private CBS cbs;
        private MvcHeuristicForCbs mvc_for_cbs;
        private CBS cbsh;

        public IndependenceDetectionAgentsGroup(ProblemInstance instance, AgentState[] allAgentsState, ISolver singleAgentSolver, ISolver groupSolver)
        {
            this.allAgentsState = allAgentsState;
            this.instance = instance.Subproblem(allAgentsState);
            this.singleAgentSolver = singleAgentSolver;
            this.groupSolver = groupSolver;
            
            var model_path = Path.Combine(Environment.CurrentDirectory, "testing-clf.xgb");

            //selectionModel = new ClassificationXGBoostLearner();
            this.selectionModel = ClassificationXGBoostModel.Load(model_path);

            this.simple = new SumIndividualCosts();
            this.epea = new EPEA_Star(this.simple);
            this.astar = new A_Star(this.simple);
            this.macbs = new CBS(this.astar, this.epea, 10);
            this.icts = new CostTreeSearchSolverOldMatching(3);
            this.cbs = new CBS(this.astar, this.epea);
            this.mvc_for_cbs = new MvcHeuristicForCbs();

            //for (int i = 0; i < astar_heuristics.Count; i++)
            //    astar_heuristics[i].Init(instance, agentList);

            this.cbsh = new CBS(astar, epea,
                mergeThreshold: -1,
                CBS.BypassStrategy.FIRST_FIT_LOOKAHEAD,
                doMalte: false,
                CBS.ConflictChoice.CARDINAL_MDD,
                this.mvc_for_cbs,
                disableTieBreakingByMinOpsEstimate: true,
                lookaheadMaxExpansions: 1,
                mergeCausesRestart: true,
                replanSameCostWithMdd: false,
                cacheMdds: false,
                useOldCost: false,
                useCAT: true);


        }

        /// <summary>
        /// Solve the group of agents together.
        /// </summary>
        /// <param name="runner"></param>
        /// <returns>true if optimal solution for the group of agents were found, false otherwise</returns>
        public bool Solve(Run runner)
        {
            ISolver relevantSolver = this.groupSolver;
            if (this.allAgentsState.Length == 1)
                relevantSolver = this.singleAgentSolver; // TODO: Consider using CBS's root trick to really get single agent paths fast. Though it won't respect illegal moves and such.
            
            relevantSolver.Setup(this.instance, runner);
            bool solved = relevantSolver.Solve();
            this.solutionCost = relevantSolver.GetSolutionCost();
            if (solved == false)
                return false;

            // Store the plan found by the solver
            this.plan = relevantSolver.GetPlan();
            this.expanded = relevantSolver.GetExpanded();
            this.generated = relevantSolver.GetGenerated();
            this.depthOfSolution = relevantSolver.GetSolutionDepth();

            // Clear memory
            relevantSolver.Clear();
            return true;
        }


        public string solverNameByIndex(double solverIndex)
        {
            switch (solverIndex)
            {
                case 0:
                    return "EPEA";
                case 1:
                    return "MA-CBS";
                case 2:
                    return "ICTS";
                case 3:
                    return "ASTAR";
                case 4:
                    return "CBS";
                case 5:
                    return "CBS-H";
                default:
                    return "WHAT?";
            }
        }

        public ISolver solverByIndex(double solverIndex, ProblemInstance instance)
        {
            List<uint> agentList = Enumerable.Range(0, instance.agents.Length).Select(x => (uint)x).ToList(); // FIXME: Must the heuristics really receive a list of uints?

            simple.Init(instance, agentList);

            switch (solverIndex)
            {
                case 0:
                    return epea;
                case 1:
                    return macbs;
                case 2:
                    return icts;
                case 3:
                    return astar;
                case 4:
                    return cbs;
                case 5:
                    return cbsh;
                default:
                    return null;
            }


        }

        /// <summary>
        /// Solve the group of agents together.
        /// </summary>
        /// <param name="runner"></param>
        /// <returns>true if optimal solution for the group of agents were found, false otherwise</returns>
        public bool SolveWithSelection(Run runner)
        {
            ISolver relevantSolver = this.groupSolver;
            if (this.allAgentsState.Length == 1)
                relevantSolver = this.singleAgentSolver; // TODO: Consider using CBS's root trick to really get single agent paths fast. Though it won't respect illegal moves and such.
            else
            {
                var pred = this.selectionModel.Predict(instance.ml_features.ToArray());

                Console.WriteLine($"-----------------AS in ID-{solverNameByIndex(pred)}-----------------");
                relevantSolver = solverByIndex(pred, this.instance);
                relevantSolver.Setup(this.instance, runner); //TODO: Move setup to ctor
            }
            bool solved = relevantSolver.Solve();
            this.solutionCost = relevantSolver.GetSolutionCost();
            if (solved == false)
                return false;

            // Store the plan found by the solver
            this.plan = relevantSolver.GetPlan();
            this.expanded = relevantSolver.GetExpanded();
            this.generated = relevantSolver.GetGenerated();
            this.depthOfSolution = relevantSolver.GetSolutionDepth();

            // Clear memory
            relevantSolver.Clear();
            return true;
        }


        /// <summary>
        /// Returns the plan for the group of agents. This is a collection of Moves for every time step until all the agents reach their goal.
        /// </summary>
        public Plan GetPlan()
        {
            return this.plan;
        }

        /// <summary>
        /// Joins this and another group to a single group with all of the agents together.
        /// </summary>
        /// <param name="other"></param>
        /// <returns>A new AgentsGroup object with the agents from both this and the other group</returns>
        public IndependenceDetectionAgentsGroup Join(IndependenceDetectionAgentsGroup other)
        {
            AgentState[] joinedAgentStates = new AgentState[allAgentsState.Length + other.allAgentsState.Length];
            this.allAgentsState.CopyTo(joinedAgentStates, 0);
            other.allAgentsState.CopyTo(joinedAgentStates, this.allAgentsState.Length);
            Array.Sort(joinedAgentStates, (x, y) => x.agent.agentNum.CompareTo(y.agent.agentNum));  // Technically could be a merge

            return new IndependenceDetectionAgentsGroup(this.instance, joinedAgentStates, this.singleAgentSolver, this.groupSolver);
        }

        /// <summary>
        /// Returns the number of agents in the group.
        /// </summary>
        public int Size()
        {
            return this.allAgentsState.Length;
        }

        public override bool Equals(object obj)
        {
            if (obj == null)
                return false;
            IndependenceDetectionAgentsGroup other = (IndependenceDetectionAgentsGroup)obj;
            return allAgentsState.SequenceEqual(other.allAgentsState);
        }

        public override int GetHashCode()
        {
            int ret = 0;
            int i = 0;
            foreach (var agentState in allAgentsState)
            {
                ret += Constants.PRIMES_FOR_HASHING[i % 10] * agentState.GetHashCode();
                i++;
            }
            return ret;
        }

        /// <summary>
        /// Tries to find a plan for this group, that will not conflict with the given plan,
        /// and still has the same solution cost as the current solution cost.
        /// This is used in the ImprovedID() method.
        /// </summary>
        /// <param name="plan"></param>
        /// <param name="runner"></param>
        /// <returns></returns>
        public bool ReplanUnderConstraints(Plan plan, Run runner, Boolean withSelection)
        {
            int oldCost = this.solutionCost;
            Plan oldPlan = this.plan;
            HashSet<TimedMove> reserved = new HashSet<TimedMove>();
            plan.AddPlanToHashSet(reserved, Math.Max(plan.GetSize(), this.plan.GetSize()));

            this.instance.parameters[IndependenceDetection.ILLEGAL_MOVES_KEY] = reserved;
            this.instance.parameters[IndependenceDetection.MAXIMUM_COST_KEY] = solutionCost;  // TODO: add IIndependenceDetectionSolver to ISolver.cs with a Setup method that takes a maxCost
            bool success;
            if (withSelection)
            {
                success = this.SolveWithSelection(runner);
            }
            else
            {
                success = this.Solve(runner); 
            }
            this.instance.parameters.Remove(IndependenceDetection.ILLEGAL_MOVES_KEY);
            this.instance.parameters.Remove(IndependenceDetection.MAXIMUM_COST_KEY);
            if (success == false)
            {
                this.solutionCost = oldCost;
                this.plan = oldPlan;
            }
            return success;
        }

        public void addGroupToCAT(Dictionary<TimedMove, List<int>> CAT, int maxTimeStep)
        {
            if (this.plan == null)
                return;
            
            for (int i = 1; i <= maxTimeStep * 2; i++) // TODO: Use the ConflictAvoidanceTable class instead
            {
                List<Move> step = plan.GetLocationsAt(i);
                foreach (Move move in step)
                {
                    TimedMove timedMove = new TimedMove(move, i);
                    if (CAT.ContainsKey(timedMove) == false)
                        CAT.Add(timedMove, new List<int>(this.allAgentsState.Length));
                    CAT[timedMove].AddRange(this.allAgentsState.Select(state => state.agent.agentNum));
                }
            }
        }

        public void removeGroupFromCAT(Dictionary<TimedMove, List<int>> CAT)
        {
            int i = 1;
            bool stop = false;
            while (stop == false)
            {
                List<Move> step = plan.GetLocationsAt(i);
                foreach (Move move in step)
                {
                    var S = new TimedMove(move, i);
                    if (CAT.ContainsKey(S))
                        CAT.Remove(S);
                    else
                    {
                        stop = true;
                        break;
                    }
                }
                i++;
            }
        }

        public override string ToString()
        {
            string ans = "group {";
            foreach (var agentState in this.allAgentsState)
            {
                ans += agentState.agent.agentNum + ", ";
            }
            ans += "}";
            return ans;
        }
    }
}
