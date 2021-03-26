using System;
using System.Linq;
using System.Collections.Generic;
using System.IO;
using System.Diagnostics;

namespace mapf
{   
    public class LazyCBS_Solver : ISolver
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
        public LazyCBS_Solver()
        {
        }

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
            return "LazyCBS";
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
            String lazy_map_problem_file = this.instance.parameters[ProblemInstance.LAZY_CBS_MAP_FILE_NAME].ToString();
            String lazy_agents_problem_file = this.instance.parameters[ProblemInstance.LAZY_CBS_AGENTS_FILE_NAME].ToString();
            String map_file = this.instance.parameters[ProblemInstance.MAP_FILE_PATH].ToString();
            String scen_file = this.instance.parameters[ProblemInstance.SCEN_FILE].ToString();
            int n_agents = (int)this.instance.parameters[ProblemInstance.N_AGENTS];

            generateMapFile(lazy_map_problem_file, map_file);
            generateAgentsFile(lazy_agents_problem_file, scen_file, n_agents);
            Console.WriteLine("Starting to solve LazyCBS problem with map {0} and agents {1}", lazy_map_problem_file, lazy_agents_problem_file);
            if (!File.Exists((String)lazy_map_problem_file))
            {
                Console.WriteLine("LazyCBS Map file not found at {0}", lazy_map_problem_file);
                return false;
            }
            if (!File.Exists((String)lazy_agents_problem_file))
            {
                Console.WriteLine("LazyCBS Agent file not found at {0}", lazy_agents_problem_file);
                return false;
            }
            process.StartInfo.FileName = "/home/local/BGU-USERS/kaduro/lazycbs/lazycbs/lazy-cbs";
            if (!File.Exists((String)process.StartInfo.FileName))
            {
                Console.WriteLine("LazyCBS exec file not found at {0}", process.StartInfo.FileName);
                return false;
            }
            process.StartInfo.Arguments = String.Format("--map {0}" +
                " --agents {1} --time_limit 300", lazy_map_problem_file, lazy_agents_problem_file);
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

        private void generateAgentsFile(string lazy_agents_problem_file, string scen_file, int n_agents)
        {
            if (!File.Exists(lazy_agents_problem_file))
            {
                System.Console.WriteLine("Creating lazycbs agents file from {0} at {1} with {2} agents",scen_file, lazy_agents_problem_file, n_agents);

                var process = new Process();
                String commandArgs = String.Format("/home/local/BGU-USERS/kaduro/lazycbs/lazycbs/lazycbs-agents-adapter.py" +
                    " {0} {1} {2}", scen_file, lazy_agents_problem_file, n_agents);
                //Console.WriteLine(commandArgs);
                process.StartInfo.FileName = "python";
                process.StartInfo.Arguments = commandArgs;
                process.StartInfo.CreateNoWindow = true;
                process.StartInfo.UseShellExecute = false;

                process.StartInfo.RedirectStandardOutput = true;
                process.OutputDataReceived += (sender, data) =>
                {
                    Console.WriteLine(data.Data);
                };
                process.StartInfo.RedirectStandardError = true;
                process.ErrorDataReceived += (sender, data) =>
                {
                    Console.WriteLine(data.Data);
                };

                process.Start();
                process.BeginOutputReadLine();
                process.BeginErrorReadLine();
                bool successInTime = process.WaitForExit(Constants.MAX_TIME);
                process.Close();
            }
        }

        private void generateMapFile(string lazy_map_problem_file, string mapFilePath)
        {
            if (!File.Exists(lazy_map_problem_file))
            {
                System.Console.WriteLine("Creating lazycbs map file from {0} at {1}", mapFilePath, lazy_map_problem_file);

                var process = new Process();
                String commandArgs = String.Format("/home/local/BGU-USERS/kaduro/lazycbs/lazycbs/lazycbs-map-adapter.py" +
                    " {0} {1}", mapFilePath, lazy_map_problem_file);
                //Console.WriteLine(commandArgs);
                process.StartInfo.FileName = "python";
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
