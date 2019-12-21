using SharpLearning.XGBoost.Learners;
using SharpLearning.XGBoost.Models;

namespace mapf
{
    /// <summary>
    /// This class is responsible for running the experiments.
    /// </summary>
    public class XGBoostTests
    {

        static void Main(string[] args)
        {
            //var learner = new ClassificationXGBoostModel();
            var model_path = "C:\\Users\\omri\\Projects\\study\\2019semB\\agentsPlanning\\MAPF\\classification\\testing-clf.xgb";
            using (var loadedModel = ClassificationXGBoostModel.Load(model_path))
            {
                //loadedModel.Predict
                System.Console.WriteLine("A");
                //var predictions = loadedModel.Predict(observations);
            }

        }
    }
}
