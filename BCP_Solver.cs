using System;
using System.Linq;
using System.Collections.Generic;
using System.IO;
using System.Diagnostics;

namespace mapf
{   
    public class BCPSolver : ISolver
    {
        protected ProblemInstance instance;
        /// <summary>
        /// Holds the cost of the solution when a solution found
        /// </summary>
        public int totalCost;
        public int numOfAgents;
        protected int maxSolutionCost;
        protected HashSet<TimedMove> illegalMoves;
        protected HashSet_U<CbsConstraint> constraints;
        protected Run runner;
        protected Plan solution;
       
        /// <summary>
        /// Default constructor.
        /// </summary>
        public BCPSolver()
        {
        }

        /// <summary>
        /// Setup the relevant data structures for a run under CBS.
        /// </summary>
        public virtual void Setup(ProblemInstance problemInstance, Run runner)
        {
            this.instance = problemInstance;
            this.runner = runner;
            this.numOfAgents = problemInstance.agents.Length;

            // Store parameters used by the Independence Detection algorithm
            if (problemInstance.parameters.ContainsKey(IndependenceDetection.MAX_COST_KEY))
                this.maxSolutionCost = (int)problemInstance.parameters[IndependenceDetection.MAX_COST_KEY];
            else
                this.maxSolutionCost = int.MaxValue;

            if (problemInstance.parameters.ContainsKey(IndependenceDetection.ILLEGAL_MOVES_KEY) &&
                ((HashSet<TimedMove>)problemInstance.parameters[IndependenceDetection.ILLEGAL_MOVES_KEY]).Count != 0)
                this.illegalMoves = (HashSet<TimedMove>)(problemInstance.parameters[IndependenceDetection.ILLEGAL_MOVES_KEY]);
            else
                this.illegalMoves = null;

        }

        /// <summary>
        /// Factory method. Creates the initial state from which the search will start. 
        /// This will be the first state to be inserted to OPEN.
        /// </summary>
        /// <returns>The root of the search tree</returns>
        protected virtual WorldState CreateSearchRoot(int minDepth = -1, int minCost = -1, MDDNode mddNode = null)
        {
            return new WorldState(this.instance.agents, minDepth, minCost, mddNode);
        }

        /// <summary>
        /// Clears the relevant data structures and variables to free memory.
        /// </summary>
        public void Clear()
        {
            this.illegalMoves = null;
        }

        public virtual String GetName()
        {
            return "BCP";
        }

        public override string ToString()
        {
            return this.GetName();
        }

        public int GetSolutionCost() { return this.totalCost; }

        public virtual void OutputStatisticsHeader(TextWriter output)
        {
        }

        /// <summary>
        /// Prints statistics of a single run to the given output. 
        /// </summary>
        public virtual void OutputStatistics(TextWriter output)
        {
        }

        public bool debug = false;

        /// <summary>
        /// Runs the algorithm until the problem is solved or memory/time is exhausted
        /// </summary>
        /// <returns>True if solved</returns>
        public virtual bool Solve()
        {
            var process = new Process();
            var bcp_problem_file = this.instance.parameters[ProblemInstance.BCP_FILE_NAME];
            Console.WriteLine("Starting to solve BCP problem {0}", bcp_problem_file);
            if (!File.Exists((String)bcp_problem_file))
            {
                Console.WriteLine("BCP Problem file not found at {0}", bcp_problem_file);
                Console.WriteLine("!!!!!!!!!!!!!!!!!!!");
                return false;
            }
            process.StartInfo.FileName = "bcp-mapf";
            process.StartInfo.Arguments = String.Format("--file={0} --agents-limit={1}", bcp_problem_file, this.instance.agents.Length);
            process.StartInfo.CreateNoWindow = true;
            process.StartInfo.UseShellExecute = false;

            process.StartInfo.RedirectStandardOutput = true;
            process.OutputDataReceived += (sender, data) => {
                //Console.WriteLine(data.Data);
            };
            process.StartInfo.RedirectStandardError = true;
            process.ErrorDataReceived += (sender, data) => {
                //Console.WriteLine(data.Data);
            };

            process.Start();
            process.BeginOutputReadLine();
            process.BeginErrorReadLine();
            bool successInTime = process.WaitForExit(Constants.MAX_TIME);

            if (!successInTime) //Timeout
            {
                process.Kill();
                totalCost = Constants.TIMEOUT_COST;
                Console.WriteLine("Out of time");
                return false;
            }
            return true;
        }

    
        /// <summary>
        /// Returns the found plan, or null if no plan was found.
        /// </summary>
        /// <returns></returns>
        public virtual Plan GetPlan()
        {
            return this.solution;
        }

        protected SinglePlan[] singlePlans;

        public virtual SinglePlan[] GetSinglePlans()
        {
            return this.singlePlans;
        }

        protected int[] singleCosts;

        public int NumStatsColumns => throw new NotImplementedException();

        public virtual int[] GetSingleCosts()
        {
            return this.singleCosts;
        }

        
        public int GetExpanded() { return -1; }
        public int GetGenerated() { return -1; }
        public int GetMaxGroupSize() { return numOfAgents; }

        public int GetSolutionDepth()
        {
            return 1;
            //throw new NotImplementedException();
        }

        public long GetMemoryUsed()
        {
            throw new NotImplementedException();
        }

        public void ClearStatistics()
        {
        }
    }
}
