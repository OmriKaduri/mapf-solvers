using System;
using System.Linq;
using System.Collections.Generic;
using System.IO;
using System.Diagnostics;

namespace mapf
{   
    /// <summary>
    /// This is an implementation of the A* algorithm for the MAPF problem.
    /// It r
    /// </summary>
    public class SATSolver : ISolver
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
        public SATSolver()
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
            return "SAT";
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
            String sat_problem_file = this.instance.parameters[ProblemInstance.SAT_FILE_NAME].ToString();
            String map_file = this.instance.parameters[ProblemInstance.MAP_FILE_PATH].ToString();
            String scen_file = this.instance.parameters[ProblemInstance.SCEN_FILE].ToString();
            int n_agents = (int)this.instance.parameters[ProblemInstance.N_AGENTS];

            generateMPFFile(sat_problem_file, map_file, scen_file, n_agents);

            Console.WriteLine("Starting to solve SAT problem {0}", sat_problem_file);
            if (!File.Exists((String)sat_problem_file))
            {
                Console.WriteLine("SAT Problem file not found at {0}",sat_problem_file);
                Console.WriteLine("!!!!!!!!!!!!!!!!!!!");
                return false;
            }
            process.StartInfo.FileName = "mapf_solver_boOX";
            process.StartInfo.Arguments = String.Format("--algorithm=smtcbs++" +
                " --output-file=solution.txt --input-file={0} --cost-limit=99999999", sat_problem_file);
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
                totalCost = Constants.NO_SOLUTION_COST;
                Console.WriteLine("Out of time");
                return false;
            }
            totalCost = 0;
            return true;
        }

        private void generateMPFFile(string sat_problem_file, string map_file, string scen_file, int n_agents)
        {
            if (!File.Exists(sat_problem_file))
            {
                System.Console.WriteLine("Creating sat file at {0}", sat_problem_file);

                var process = new Process();
                //Console.WriteLine("Converting {0} to SAT file", fileName);

                String commandArgs = String.Format("--input-movi-map-file={0}" +
                    " --input-movi-scen-file={1}" +
                    " --output-mpf-file={2}" +
                    " --N-agents={3}", map_file, scen_file, sat_problem_file, n_agents);
                //Console.WriteLine(commandArgs);
                process.StartInfo.FileName = "moviscen_convert_boOX";
                process.StartInfo.Arguments = commandArgs;
                process.StartInfo.CreateNoWindow = true;
                process.StartInfo.UseShellExecute = false;

                process.StartInfo.RedirectStandardOutput = true;
                process.OutputDataReceived += (sender, data) =>
                {
                    //Console.WriteLine(data.Data);
                };
                process.StartInfo.RedirectStandardError = true;
                process.ErrorDataReceived += (sender, data) =>
                {
                    //Console.WriteLine(data.Data);
                };

                process.Start();
                process.BeginOutputReadLine();
                process.BeginErrorReadLine();
                bool successInTime = process.WaitForExit(Constants.MAX_TIME);
                process.Close();
            }
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

        public int NumStatsColumns
        {
            get
            {
                return 0;
            }
        }

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
