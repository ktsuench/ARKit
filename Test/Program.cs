using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Test
{
    class Program
    {
        static void Main(string[] args)
        {
      // Test 1
      //ARKit.Test.HelloWord();

      // Test 2
      // ARKit.Test.CaptureAndShow();

      // Test 3
      // ARKit.Test.RunChessboardDemo(new ARKit.Size(4, 7));

      // Test 4
      // ARKit.Test.CalibrateCamera(new ARKit.Size(4, 7), 30);

      // Test 5
      // ARKit.Test.ReadIntrinsicsFile();

      // Test 6
      // ARKit.Test.GetFeaturePoints("agenda.jpg", "keypoints.yml");

      // Test 7
      // ARKit.Test.ReadFeaturePoints("keypoints.yml");

      // Test 8
      // ARKit.Test.MatchFeatures("agenda.jpg", "keypoints.yml", new ARKit.Size(720, 1080));

      // Test 9
      //ARKit.Test.TrackFeatures("agenda_diff_lighting_lowdef.jpg", "keypoints.yml", new ARKit.Size(720, 1080));

      //Test 10
      string original = "images/simpsons.jpg";
      string keypoints_file_original = "images/original_keypoints.yml";
      string matched = "images/match.jpg";
      string tracked = "images/track.jpg";
      ARKit.Test.TestProjectionMatrix(original, keypoints_file_original, matched, tracked);

            //PAARTH - TESTING
            // testing feature extraction with AKAZE

      //TEST 1: LIGHT - used 1280 x 720 res
      //string image1 = "ARKit tests/Light_Amount/high_light.jpg";
      // string image2 = "ARKit tests/Light_Amount/low_light.jpg"; ;
      //string keypoints_1 = "ARKit tests/Light_Amount/high_light.yml";
      //string keypoints_2 = "ARKit tests/Light_Amount/low_light.yml";
      //ARKit.FeaturePoints.ComputeAndSave(image1, keypoints_1);
      //ARKit.FeaturePoints.ComputeAndSave(image2, keypoints_2);

      ////TEST 2: RESOLUTION OF CAMERA
      //string image_res1 = "ARKit tests/Resolution/1280_720.jpg";
      //string image_res2 = "ARKit tests/Resolution/640_480.jpg";
      //string image_res3 = "ARKit tests/Resolution/160_120.jpg";
      //string image_res4 = "ARKit tests/Resolution/176_144.jpg";
      //string image_res5 = "ARKit tests/Resolution/320_240.jpg";
      //string image_res6 = "ARKit tests/Resolution/352_288.jpg";

      //string keypoints_res1 = "ARKit tests/Light_Amount/1280_720.yml";
      //string keypoints_res2 = "ARKit tests/Light_Amount/640_480.yml";
      //string keypoints_res3 = "ARKit tests/Light_Amount/160_120.yml";
      //string keypoints_res4 = "ARKit tests/Light_Amount/176_144.yml";
      //string keypoints_res5 = "ARKit tests/Light_Amount/320_240.yml";
      //string keypoints_res6 = "ARKit tests/Light_Amount/352_288.yml";

      //ARKit.FeaturePoints.ComputeAndSave(image_res1, keypoints_res1);
      //ARKit.FeaturePoints.ComputeAndSave(image_res2, keypoints_res2);
      //ARKit.FeaturePoints.ComputeAndSave(image_res3, keypoints_res3);
      //ARKit.FeaturePoints.ComputeAndSave(image_res4, keypoints_res4);
      //ARKit.FeaturePoints.ComputeAndSave(image_res5, keypoints_res5);
      //ARKit.FeaturePoints.ComputeAndSave(image_res6, keypoints_res6);

      //testing feature matching
      //test with high light + 1280x720

      //provide camera resolution
      //ARKit.Size sz = new ARKit.Size(1280, 720);
      //ARKit.Test.MatchFeatures(image_res2, keypoints_res2, sz);

      ////test with low light + 1280x720
      //ARKit.Test.MatchFeatures(image2, keypoints_2, sz);




    }
  }
}
