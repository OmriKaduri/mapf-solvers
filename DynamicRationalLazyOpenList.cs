﻿using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;

namespace mapf
{
    public class DynamicRationalLazyOpenList : OpenList<WorldState>
    {
        public Run runner;
        public IBoundedLazyHeuristic<WorldState> expensive;
        protected int lastF;
        protected int skips;
        protected int accSkips;
        protected (double success, double failure, double ph, double uses)[] capData;
        protected (double success, double failure, double ph, double uses)[] accCapData;
        protected static readonly double MOVING_AVERAGE_FACTOR = 1;
        protected static readonly int NUM_CAPS = 27;
        private double expandStartTime;
        protected double sumExpandTimes;
        protected int numExpands;
        protected double accSumExpandTimes;
        protected int accNumExpands;

        public DynamicRationalLazyOpenList(ISolver user, IBoundedLazyHeuristic<WorldState> expensive)
            : base(user)
        {
            this.expensive = expensive;
            this.ClearPrivateStatistics();
            this.ClearPrivateAccumulatedStatistics();
        }

        public override string GetName()
        {
            return $"Dynamic rational lazy open list with heuristic {this.expensive.GetName()}";
        }

        protected new void ClearPrivateStatistics()
        {
            this.lastF = -1;

            this.numExpands = 0;
            this.sumExpandTimes = 0;
            this.skips = 0;
            this.capData = new (double success, double failure, double ph, double uses)[NUM_CAPS];
        }

        protected new void ClearPrivateAccumulatedStatistics()
        {
            this.accNumExpands = 0;
            this.accSumExpandTimes = 0;
            this.accSkips = 0;
            this.accCapData = new (double success, double failure, double ph, double uses)[NUM_CAPS];
        }

        public override string ToString()
        {
            return $"DynamicRationalLazyOpenList/{this.expensive}";
        }

        //bool runOracle = true;

        public override WorldState Remove()
        {
            WorldState node;

            if (this.lastF != -1)
            {
                this.numExpands++;
                double expandFinishTime = this.runner.ElapsedMilliseconds();
                this.sumExpandTimes += expandFinishTime - this.expandStartTime;
            }

            if (base.Count == 1) // Can happen more than once with the same node if partial expansion pushes it back into the open list
            {
                node = base.Remove();
                goto finish;
            }

            // There are alternatives to the lowest cost node in the open list, try to postpone expansion of it:
            float branchingFactor = ((A_Star)this.user).GetEffectiveBranchingFactor(); // We know the solver is an A* variant.
            const double binaryHeapTau = 0.073359375; // microseconds. From empirical experiments with this infra on my computer.
            double logN = Math.Log(this.heap.Count, 2); // Removals from and insertions to the queue cost practically zero.
            double t0 = binaryHeapTau * logN; // TODO: Measure this directly?
            double overhead = 0.023 * this.Peek().allAgentsState.Length; // in milliseconds. Empirical lowest estimate. The cost of a zero-timeout CBSH run wasn't simply linear with the number of agents for some reason.

            while (true)
            {
                WorldState next;
                node = base.Remove();

                if (node.GoalTest() == true || // Can't improve the h of the goal
                    this.runner.ElapsedMilliseconds() > Constants.MAX_TIME) // No time to continue improving H.
                {
                    if (node.g + node.h < this.lastF) // This can happen if the last removed node had many runs of the expensive heuristic, which this node didn't yet have.
                    {
                        int newH = this.lastF - node.g;
                        node.hBonus += newH - node.h;
                        node.h = newH; // Just so we don't throw an inconsistency exception
                    }
                    break;
                }

                if (node.hBonus > 0) // Improving the h will be more difficult than usual - don't try
                {
                    this.skips++; // TODO: Separate statistic?
                    break;
                }

                int selectedCapExponent = -1;

                //if (runOracle)
                //{
                //    this.runner.StartOracle();
                //
                //    next = base.Peek();
                //    int targetH = next.g + next.h + 1 - node.g;
                //
                //    Stopwatch watch = Stopwatch.StartNew();
                //    WorldStateForPartialExpansion nodeCopy = new WorldStateForPartialExpansion((WorldStateForPartialExpansion)node); // So the solution won't leak to the node when we try to solve it.
                //    double expensiveCallStartTime = watch.Elapsed.TotalMilliseconds;
                //    // Using a separate statistic for the oracle to keep the stats clean
                //    uint expensiveEstimate = ((IBoundedLazyHeuristic<IBinaryHeapItem>)this.runner.astar_heuristics[this.runner.astar_heuristics.Count - 1]).h(
                //                                                                    nodeCopy, targetH, -1, int.MaxValue, false);
                //    double expensiveCallTotalTime = watch.Elapsed.TotalMilliseconds - expensiveCallStartTime;
                //
                //    int lowestCapThatWouldHaveWorked = (int)Math.Ceiling(Math.Log(expensiveCallTotalTime * 1000, 2));
                //    //Console.WriteLine("Lowest cap that would have worked:{0}", (1<<lowestCapThatWouldHaveWorked)/1000.0);
                //    for (int j = 0; (j < lowestCapThatWouldHaveWorked) && (j < NUM_CAPS); ++j)
                //    {
                //        this.capData[j].failure += 1;
                //        this.capData[j].ph = 0; // Correct for this specific node.
                //                                                                 // All expanded nodes either have the oracle run on them or they don't, so this value won't leak to other nodes.
                //    }
                //    for (int j = lowestCapThatWouldHaveWorked; j < NUM_CAPS; ++j)
                //    {
                //        this.capData[j].success = +1;
                //        this.capData[j].ph = 1;
                //    }
                //
                //    this.runner.StopOracle();
                //}

                // DRLA* calculation (derived from Tolpin et al. RLA* formula):
                double tExpand = (this.sumExpandTimes / this.numExpands) * 1000; // in microseconds

                int nextCapExponentAboveTExpand;
                if (tExpand != 0)
                {
                    nextCapExponentAboveTExpand = (int)Math.Ceiling(Math.Log(tExpand, 2));
                    nextCapExponentAboveTExpand = Math.Min(nextCapExponentAboveTExpand, NUM_CAPS - 1);
                }
                else
                    nextCapExponentAboveTExpand = 0;

                // Force the heuristic to be tried if it's never been tried before at a cap that's one above Texpand.
                // This is needed because we initially believe all caps are useless, so the caps that are too short won't be tried
                // and fail. This has the added benefit that after running, we'll have an informed belief for all lower caps and possibly
                // all higher caps too.
                if (this.capData[nextCapExponentAboveTExpand].success +
                    this.capData[nextCapExponentAboveTExpand].failure == 0) // This cap was never tried before, and it might be good.
                {
                    selectedCapExponent = nextCapExponentAboveTExpand;
                }
                else
                {
                    double[] expectedRegret = new double[NUM_CAPS]; // in microseconds
                    double lowestRegret = double.MaxValue;

                    //double tempStartTime = ((DyanamicLazyCbsh)this.expensive).runner.ElapsedMilliseconds();

                    // Computing the expected regret from choosing each cap, and finding the cap that gives the lowest expected regret
                    for (int i = 0; i < NUM_CAPS; ++i)
                    {
                        expectedRegret[i] = 0;
                        // Going over the probabilities that the caps would be helpful
                        for (int j = 0; j <= i; ++j)
                        {
                            double heuristicOnlyHelpfulAfter = 1 << j; // microseconds
                            double probabilityForThat;
                            if (j != 0)
                                probabilityForThat = this.capData[j].ph - this.capData[j - 1].ph; // The PH_IND points to the probability that the given cap would succeed at all, including the probability that it would've worked for lower caps.
                            else
                                probabilityForThat = this.capData[j].ph;
                            expectedRegret[i] += (0.75 * heuristicOnlyHelpfulAfter + t0 - Math.Min(0.75 * heuristicOnlyHelpfulAfter + t0, tExpand)) * probabilityForThat;  // The cap is an upper bound on t2. A closer estimate is 0.75*cap (since 0.5*cap is the previous cap).
                        }
                        double cap = 1 << i;
                        for (int j = i + 1; j < NUM_CAPS; ++j)
                        {
                            double heuristicOnlyHelpfulAfter = 1 << j; // microseconds
                            double probabilityForThat = this.capData[j].ph - this.capData[j - 1].ph;
                            expectedRegret[i] += (cap + tExpand - Math.Min(0.75 * heuristicOnlyHelpfulAfter + t0, tExpand)) * probabilityForThat;
                        }
                        // The case where even the highest cap fails:
                        int last = NUM_CAPS - 1;
                        expectedRegret[i] += cap * (1 - this.capData[last].ph);

                        if (expectedRegret[i] < lowestRegret)
                        {
                            lowestRegret = expectedRegret[i];
                            selectedCapExponent = i;
                        }
                    }
                }

                double millisCap = ((double)(1 << selectedCapExponent)) / 1000;

                //double tempFinishTime = ((DyanamicLazyCbsh)this.expensive).runner.ElapsedMilliseconds();
                //Console.WriteLine("calculations time:{0}", tempFinishTime - tempStartTime);
                //Console.WriteLine("millisCap:{0}", millisCap);
                //Console.WriteLine("Texpand:{0}={1}/{2}", tExpand, this.sumExpandTimes * 1000, this.numExpands);
                //Console.WriteLine("T0:{0}", t0);
                //for (int i=0 ; i<NUM_CAPS ; ++i)
                    //Console.WriteLine("Ph for cap {0}:{1}={2}/{3}", ((double)(1 << i)) / 1000, this.capData[i].ph, this.capData[i].success, this.capData[i].success + this.capData[i].failure);
                //for (int i = 0; i < NUM_CAPS; ++i)
                    //Console.WriteLine("Expected regret for cap {0}:{1}", ((double)(1 << i)) / 1000, expectedRegret[i]/1000.0);

                if (node.g + node.h < lastF) // Must improve the heuristic estimate to be consistent
                    millisCap = Double.MaxValue;

                bool success = false;
                if (millisCap > overhead)  // Worth running the expensive heuristic
                {
                    next = this.Peek();
                    int targetH = node.GetTargetH(next.f + 1);

                    double expensiveCallStartTime = this.runner.ElapsedMilliseconds();
                    int expensiveEstimate = (int)this.expensive.h(node, targetH, -1, (int)(expensiveCallStartTime + millisCap), false);
                    double expensiveCallTotalTime = this.runner.ElapsedMilliseconds() - expensiveCallStartTime;

                    bool nodeSolved = node.GoalTest();

                    if (expensiveEstimate > node.h) // Node may have inherited a better estimate from
                                                    // its parent so this check is necessary for failures
                    {
                        node.hBonus += expensiveEstimate - node.h;
                        node.h = expensiveEstimate; // If this wasn't a success, the assignment here serves to force the next heuristic call
                                                    // to search deeper, since we're always consistent.
                    }

                    this.capData[selectedCapExponent].uses++;

                    success = node.CompareTo(next) == 1; // Node is not the smallest F anymore - re-insert it into the open list

                    // Update capData:
                    if (success || nodeSolved)
                    {
                        int lowestCapThatWouldHaveWorked = (int)Math.Ceiling(Math.Log(expensiveCallTotalTime * 1000, 2));
                        //Console.WriteLine("Lowest cap that would have worked:{0}", (1<<lowestCapThatWouldHaveWorked)/1000.0);
                        for (int j = 0; (j < lowestCapThatWouldHaveWorked) && (j < NUM_CAPS); ++j)
                        {
                            this.capData[j].failure += 1;
                            this.capData[j].ph = /*(1 - MOVING_AVERAGE_FACTOR) * this.capData[j].ph + MOVING_AVERAGE_FACTOR * */ 
                                                        this.capData[j].success / (this.capData[j].success + this.capData[j].failure);
                        }
                        for (int j = lowestCapThatWouldHaveWorked; j < NUM_CAPS; ++j)
                        {
                            this.capData[j].success += 1;
                            this.capData[j].ph = /*(1 - MOVING_AVERAGE_FACTOR) * this.capData[j].ph + MOVING_AVERAGE_FACTOR * */ 
                                                        this.capData[j].success / (this.capData[j].success + this.capData[j].failure);
                        }
                    }
                    else
                    {
                        int effectiveCap = (int)Math.Floor(Math.Log(expensiveCallTotalTime * 1000, 2)); // Heuristic may have gone over its timeout
                        for (int j = 0; (j <= effectiveCap) && (j < NUM_CAPS); ++j)
                        {
                            this.capData[j].failure += 1;
                            this.capData[j].ph = /*(1 - MOVING_AVERAGE_FACTOR) * this.capData[j].ph + MOVING_AVERAGE_FACTOR * */ 
                                                        this.capData[j].success / (this.capData[j].success + this.capData[j].failure); // This is only necessary for the first runs so the Ph won't jitter too much.
                        }
                        // No info on whether a larger cap would have worked.
                    }
                }
                else
                    this.skips++;

                if (success || node.g + node.h < lastF) // Never be inconsistent - don't return nodes with lower F than before. Try searching the node again.
                {
                    this.Add(node);
                }
                else
                {
                    // Node is still less than or equal to all items in the open list (and its h would look consistent to A*).
                    // This can be because of many reasons:
                    // - The search ended because of a timeout
                    // - The search ended because a goal was found (good!)
                    // - The search ended because the CBS search was too costly
                    break;
                }
            }

            finish:
            this.lastF = node.g + node.h;
            this.expandStartTime = this.runner.ElapsedMilliseconds();
            return node;
        }

        public override void OutputStatisticsHeader(TextWriter output)
        {
            base.OutputStatisticsHeader(output);

            output.Write($"{this} Skips");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write($"{this} Average Texpand (ms)");
            output.Write(Run.RESULTS_DELIMITER);

            for (int i = 0; i < NUM_CAPS; ++i)
            {
                output.Write($"{this} Ph(2**{i} microseconds)");
                output.Write(Run.RESULTS_DELIMITER);
                output.Write($"{this} 2**{i} microseconds cap uses)");
                output.Write(Run.RESULTS_DELIMITER);
            }

            this.expensive.OutputStatisticsHeader(output);
        }

        public override void OutputStatistics(TextWriter output)
        {
            base.OutputStatistics(output);

            double Texpand;
            if (this.numExpands != 0)
                Texpand = this.sumExpandTimes / this.numExpands;
            else
                Texpand = -1;

            Console.WriteLine(this.ToString() + " Skips: {0}", this.skips);
            Console.WriteLine(this.ToString() + " Average Texpand (ms): {0}", Texpand);

            output.Write(this.skips + Run.RESULTS_DELIMITER);
            output.Write(Texpand + Run.RESULTS_DELIMITER);

            // Phelfuls:
            for (int i = 0; i < NUM_CAPS; ++i)
            {
                Console.WriteLine(this.ToString() + " Ph(2**{0} microseconds): {1}", i, this.capData[i].ph);
                Console.WriteLine(this.ToString() + " 2**{0} microseconds cap uses: {1}", i, this.capData[i].uses);

                output.Write(this.capData[i].ph);
                output.Write(Run.RESULTS_DELIMITER);
                output.Write(this.capData[i].uses);
                output.Write(Run.RESULTS_DELIMITER);
            }

            this.expensive.OutputStatistics(output);
        }

        public override int NumStatsColumns
        {
            get
            {
                return 2 + 2 * NUM_CAPS + base.NumStatsColumns + this.expensive.NumStatsColumns;
            }
        }

        public override void ClearStatistics()
        {
            base.ClearStatistics();

            this.ClearPrivateStatistics();

            this.expensive.ClearStatistics();
        }

        public override void ClearAccumulatedStatistics()
        {
            base.ClearAccumulatedStatistics();

            this.ClearPrivateAccumulatedStatistics();

            this.expensive.ClearAccumulatedStatistics();
        }

        public override void AccumulateStatistics()
        {
            base.AccumulateStatistics();

            this.accSkips += this.skips;
            this.accNumExpands += this.numExpands;
            this.accSumExpandTimes += this.sumExpandTimes;

            for (int i = 0; i < this.capData.GetLength(0); ++i)
            {
                this.accCapData[i].success += this.capData[i].success;
                this.accCapData[i].success += this.capData[i].failure;
                this.accCapData[i].success += this.capData[i].ph;
                this.accCapData[i].success += this.capData[i].uses;
            }

            this.expensive.AccumulateStatistics();
        }

        public override void OutputAccumulatedStatistics(TextWriter output)
        {
            base.OutputAccumulatedStatistics(output);

            double accTexpand;
            if (this.accNumExpands != 0)
                accTexpand = this.accSumExpandTimes / this.accNumExpands;
            else
                accTexpand = -1;

            Console.WriteLine(this.ToString() + " Accumulated Skips: {0}", this.accSkips);
            Console.WriteLine(this.ToString() + " Accumulated Texpand (ms): {0}", accTexpand);

            output.Write(this.accSkips + Run.RESULTS_DELIMITER);
            output.Write(accTexpand + Run.RESULTS_DELIMITER);

            // Phelfuls:
            for (int i = 0; i < NUM_CAPS; ++i)
            {
                double denom = this.accCapData[i].success + this.accCapData[i].failure;
                if (denom != 0)
                    this.accCapData[i].ph = this.accCapData[i].success / denom;
                else
                    this.accCapData[i].ph = -1;
                Console.WriteLine(this.ToString() + " Accumulated Ph(2**{0} microseconds): {1}", i, this.accCapData[i].ph);
                Console.WriteLine(this.ToString() + " Accumulated 2**{0} microseconds cap uses: {1}", i, this.accCapData[i].uses);

                output.Write(this.accCapData[i].ph);
                output.Write(Run.RESULTS_DELIMITER);
                output.Write(this.accCapData[i].uses);
                output.Write(Run.RESULTS_DELIMITER);
            }

            this.expensive.OutputAccumulatedStatistics(output);
        }
    }
}
