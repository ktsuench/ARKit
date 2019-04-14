using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Features2D;
using Emgu.CV.Structure;
using Emgu.CV.Util;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Accord.Math;
using System.Media;
using MathNet.Numerics;

namespace ARKit
{
  // taken from https://stackoverflow.com/questions/32255440/how-can-i-get-and-set-pixel-values-of-an-emgucv-mat-image
  public static class MatExtension
  {
    public static dynamic GetValue(this Mat mat, int row, int col)
    {
      var value = CreateElement(mat.Depth);
      System.Runtime.InteropServices.Marshal.Copy(mat.DataPointer + (row * mat.Cols + col) * mat.ElementSize, value, 0, 1);
      return value[0];
    }

    public static void SetValue(this Mat mat, int row, int col, dynamic value)
    {
      var target = CreateElement(mat.Depth, value);
      System.Runtime.InteropServices.Marshal.Copy(target, 0, mat.DataPointer + (row * mat.Cols + col) * mat.ElementSize, 1);
    }

    private static dynamic CreateElement(DepthType depthType, dynamic value)
    {
      var element = CreateElement(depthType);
      element[0] = value;
      return element;
    }

    private static dynamic CreateElement(DepthType depthType)
    {
      if (depthType == DepthType.Cv8S)
      {
        return new sbyte[1];
      }
      if (depthType == DepthType.Cv8U)
      {
        return new byte[1];
      }
      if (depthType == DepthType.Cv16S)
      {
        return new short[1];
      }
      if (depthType == DepthType.Cv16U)
      {
        return new ushort[1];
      }
      if (depthType == DepthType.Cv32S)
      {
        return new int[1];
      }
      if (depthType == DepthType.Cv32F)
      {
        return new float[1];
      }
      if (depthType == DepthType.Cv64F)
      {
        return new double[1];
      }
      return new float[1];
    }
  }

  public static class Memory
  {
    public static Mat Frame { get; set; }
  }

  public class Frame
  {
    private readonly int _height, _width;
    private readonly byte[] _image;

    public Frame(int height, int width, byte[] image)
    {
      this._height = height;
      this._width = width;
      this._image = image;
    }

    public int Height { get => this._height; }
    public int Width { get => this._width; }
    public byte[] Image { get => this._image; }
  }

  public class Size
  {
    private System.Drawing.Size _size;
    private int _height;
    private int _width;

    public Size(int width, int height)
    {
      this._width = width;
      this._height = height;
      this._size = new System.Drawing.Size(width, height);
    }

    public int Height { get => this._height; }
    public int Width { get => this._width; }

    public System.Drawing.Size Dims => this._size;
  }

  // based on 2nd answer https://answers.unity.com/questions/52368/emgucv-inside-unity.html
  public class Camera
  {
    private readonly VideoCapture cap;
    private readonly bool unity;

    public Camera(int cameraId = 0, Size size = null, bool unity = true)
    {
      this.cap = new VideoCapture(cameraId + CaptureType.Msmf);

      if (this.cap.IsOpened)
      {
        if (size.Dims.Height != 0 && size.Dims.Width != 0)
        {
          this.cap.SetCaptureProperty(CapProp.FrameHeight, size.Dims.Height);
          this.cap.SetCaptureProperty(CapProp.FrameWidth, size.Dims.Width);
        }
      }
      else
        throw new Exception("Cannot open camera with the given params");

      this.unity = unity;
    }

    public Camera(Size size, bool unity = true) : this(0, size, unity) { }
    public Camera(bool unity) : this(null, unity) { }

    public Frame GetNextFrame()
    {
      Memory.Frame = this.cap.QueryFrame();
      using (Image<Bgr, byte> nextFrame = Memory.Frame.ToImage<Bgr, byte>())
      {
        System.Drawing.Bitmap currentFrame = nextFrame.ToBitmap();
        MemoryStream m = new MemoryStream();
        System.Drawing.Imaging.ImageFormat format =
          !this.unity ? System.Drawing.Imaging.ImageFormat.Bmp : currentFrame.RawFormat;

        /*
         * was throwing encoder cannot be null for formats that are MemoryBmp,
         * found and fixed by referring to https://stackoverflow.com/questions/25242728/image-save-throws-exception-value-cannot-be-null-r-nparameter-name-encoder
         */
        currentFrame.Save(m, format);

        return new Frame(currentFrame.Height, currentFrame.Width, m.ToArray());
      }
    }

    public void Release()
    {
      this.cap.Dispose();
    }
  }

  /*public class Extrinsics
  {
    private readonly Image<Bgr, byte> image;

    public Extrinsics(Frame frame) {
      this.image = new Image<Bgr, Byte>(frame.getWidth(), frame.getHeight())
      {
        Bytes = frame.getImage()
      };
    }

    public UnityEngine.Matrix4x4 getMatrix()
    {
      UnityEngine.Vector3 offset = this.image.
    }
  }*/

  public class FeaturePoints
  {
    private static readonly float ACCEPTABLE_TRACKING_AVERAGE_ERROR = 20.0f;
    private static readonly float INLIER_THRESHOLD = 2.5f;
    private static readonly float INLIER_USABLE_THRESHOLD = 0.3f;
    private static readonly int KTH_NEAREST_NEIGHBOUR = 2;
    private static readonly int MAX_PYRAMID_LEVELS = 3; // default used in OpenCV
    private static readonly int MATCHES_REQUIRED = 10;
    private static readonly float NN_MATCH_RATIO = 0.8f;
    private static readonly float RANSAC_REPROJECTION_THRESHOLD = 3f; // default
    // either finish by 30 iterations or search window moved less than epsilon of 0.01
    private static readonly System.Drawing.Size SEARCH_WINDOW_SIZE =
      new System.Drawing.Size(21, 21); // default used in OpenCV
    private static readonly MCvTermCriteria TERMINATION_CRITERIA =
      new MCvTermCriteria(30, 0.01); // default used in OpenCV

    public enum FeatureState { MATCHING, TRACKING };

    private readonly VectorOfPointF BORDER;

    private readonly Mat _DESCRIPTORS;
    private readonly VectorOfKeyPoint _KEYPOINTS;
    private readonly System.Drawing.Size _SIZE;
    private readonly bool _UNITY;

    private VectorOfPointF _borderPoints;
    private Mat _homographyMatchMat;
    private Mat _homographyTrackMat;
    private int _inliers;
    private double _inlierRatio;
    private int _matches;
    private VectorOfPointF _previousBorderPoints;
    private Mat _previousFrame;
    private VectorOfPointF _previousPoints;
    private bool _replacePreviousBorderPoints;
    private Mat _rotationMatrix;
    private FeatureState _state;
    private float _trackerAvgErr;
    private Single[] _trackerErr;
    private byte[] _trackerStatus;
    private Mat _translationVector;
    private bool _useExtrinsicGuessForPnP = false;

    public FeaturePoints(System.Drawing.Size size, VectorOfKeyPoint keypoints,
      Mat descriptors, bool unity = true)
    {
      this._DESCRIPTORS = descriptors;
      this._KEYPOINTS = keypoints;
      this._SIZE = size;
      this._UNITY = unity;

      BORDER = new VectorOfPointF(new System.Drawing.PointF[] {
        new System.Drawing.PointF(0, 0),
        new System.Drawing.PointF(this._SIZE.Width, 0),
        new System.Drawing.PointF(0, this._SIZE.Height),
        new System.Drawing.PointF(this._SIZE.Width, this._SIZE.Height),
      });

      this._borderPoints = new VectorOfPointF();
      this._homographyMatchMat = new Mat();
      this._homographyTrackMat = new Mat();
      this._inliers = 0;
      this._inlierRatio = 0;
      this._matches = 0;
      this._previousBorderPoints = new VectorOfPointF(BORDER.ToArray());
      this._previousFrame = new Mat();
      this._previousPoints = new VectorOfPointF();
      this._replacePreviousBorderPoints = false;
      this._rotationMatrix = new Mat();
      this._trackerAvgErr = 0;
      this._trackerStatus = new byte[] { };
      this._trackerErr = new float[] { };
      this._translationVector = new Mat();
      this._state = FeatureState.MATCHING;
    }

    public Mat Descriptors { get => this._DESCRIPTORS; }
    public int Inliers { get => this._inliers; }
    public double InlierRatio { get => this._inlierRatio; }
    public VectorOfKeyPoint KeyPoints { get => this._KEYPOINTS; }
    public int Matches { get => this._matches; }
    public System.Drawing.Size Size { get => this._SIZE; }
    public float TrackerAverageError { get => this._trackerAvgErr; }
    public FeatureState State { get => this._state; }

    public static void ComputeAndSave(string imageFilePath, string keypointsFilePath)
    {
      VectorOfKeyPoint itemKeypoints = new VectorOfKeyPoint();
      Mat itemDescriptors = new Mat(), image = CvInvoke.Imread(imageFilePath);

      CvInvoke.CvtColor(image, image, ColorConversion.Rgb2Gray);

      using (AKAZE akaze = new AKAZE())
      {
        akaze.DetectAndCompute(image, null, itemKeypoints, itemDescriptors, false);
      }

      using (FileStorage fs = new FileStorage(keypointsFilePath,
        FileStorage.Mode.Write | FileStorage.Mode.FormatYaml))
      {
        fs.Write(image.Height, "objectHeight");
        fs.Write(image.Width, "objectWidth");
        fs.Write(itemKeypoints.Size, "numberOfKeypoints");
        for (int i = 0; i < itemKeypoints.ToArray().Length; i++)
        {
          MKeyPoint m = itemKeypoints.ToArray()[i];
          fs.Write(m.Angle, "keypoint-" + i + "-angle");
          fs.Write(m.ClassId, "keypoint-" + i + "-classId");
          fs.Write(m.Octave, "keypoint-" + i + "-octave");
          fs.Write(m.Point.X, "keypoint-" + i + "-point-x");
          fs.Write(m.Point.Y, "keypoint-" + i + "-point-y");
          fs.Write(m.Response, "keypoint-" + i + "-response");
          fs.Write(m.Size, "keypoint-" + i + "-size");
        }
        fs.Write(itemDescriptors, "descriptors");
        fs.ReleaseAndGetString();
      }
    }

    public static FeaturePoints ReadData(string filepath, bool unity = true)
    {
      System.Drawing.Size size = new System.Drawing.Size();
      Mat descriptors = new Mat();
      List<MKeyPoint> keypoints = new List<MKeyPoint>();

      using (FileStorage fs = new FileStorage(
        filepath, FileStorage.Mode.Read | FileStorage.Mode.FormatYaml))
      {
        size.Height = fs.GetNode("objectHeight").ReadInt();
        size.Width = fs.GetNode("objectWidth").ReadInt();
        int numKeypoints = fs.GetNode("numberOfKeypoints").ReadInt();
        for (int i = 0; i < numKeypoints; i++)
        {
          MKeyPoint m = new MKeyPoint
          {
            Angle = fs.GetNode("keypoint-" + i + "-angle").ReadFloat(),
            ClassId = fs.GetNode("keypoint-" + i + "-classId").ReadInt(),
            Octave = fs.GetNode("keypoint-" + i + "-octave").ReadInt(),
            Point = new System.Drawing.PointF(
              fs.GetNode("keypoint-" + i + "-point-x").ReadFloat(),
              fs.GetNode("keypoint-" + i + "-point-y").ReadFloat()),
            Response = fs.GetNode("keypoint-" + i + "-response").ReadFloat(),
            Size = fs.GetNode("keypoint-" + i + "-size").ReadFloat()
          };

          keypoints.Add(m);
        }
        fs.GetNode("descriptors").ReadMat(descriptors);
      }

      return new FeaturePoints(
        size, new VectorOfKeyPoint(keypoints.ToArray()), descriptors, unity);
    }

    public bool GetHomography(out Mat homography)
    {
      Matrix<double> Hm, Ht;

      homography = null;

      if (this._homographyMatchMat.IsEmpty)
        return false;
      else
      {
        Hm = new Matrix<double>(
          this._homographyMatchMat.Rows, this._homographyMatchMat.Cols);

        this._homographyMatchMat.CopyTo(Hm);

        homography = new Mat();

        if (this._homographyTrackMat.IsEmpty)
          Hm.Mat.ConvertTo(homography, DepthType.Cv32F);
        else
        {
          Ht = new Matrix<double>(
            this._homographyTrackMat.Rows, this._homographyTrackMat.Cols);

          this._homographyTrackMat.CopyTo(Ht);

          (Hm * Ht).Mat.ConvertTo(homography, DepthType.Cv32F);
        }

        return true;
      }
    }

    private int CheckHomography(VectorOfPointF trainCoords, VectorOfPointF queryCoords, out Mat homography)
    {
      // VectorOfPointF inliers1 = new VectorOfPointF();
      // VectorOfPointF inliers2 = new VectorOfPointF();
      Matrix<double> homographyMat;
      int inliers = 0;

      // determine homography matrix
      homography = CvInvoke.FindHomography(trainCoords, queryCoords,
        HomographyMethod.Ransac, RANSAC_REPROJECTION_THRESHOLD);
      homographyMat = new Matrix<double>(homography.Rows, homography.Cols);
      homography.CopyTo(homographyMat);

      if (!homography.Size.IsEmpty)
        /*
         * check that the matches fit the homography model
         * by transforming the key points of the item and
         * comparing with the detected key points in the image
         * of where the item should be
         */
        for (int i = 0; i < trainCoords.Size; i++)
        {
          Mat col = Mat.Ones(3, 1, DepthType.Cv64F, 3);
          Matrix<double> colm = new Matrix<double>(col.Rows, col.Cols);
          col.SetValue(0, 0, trainCoords[i].X);
          col.SetValue(1, 0, trainCoords[i].Y);

          col.CopyTo(colm);
          colm = homographyMat * colm;
          colm /= colm[2, 0];

          double dist = Math.Sqrt(
            Math.Pow(colm[0, 0] - queryCoords[i].X, 2) +
            Math.Pow(colm[1, 0] - queryCoords[i].Y, 2));

          if (dist < INLIER_THRESHOLD)
            /*{
              inliers1.Push(new System.Drawing.PointF[] { trainCoords[i] });
              inliers2.Push(new System.Drawing.PointF[] { queryCoords[i] });
            }*/
            inliers++;
        }

      return inliers;
    }

    public void ComputeAndMatch()
    {

      VectorOfKeyPoint imageKeypoints = new VectorOfKeyPoint();
      Mat imageDescriptors = new Mat(), image = Memory.Frame.Clone();
      // DMatch type explanation
      // https://stackoverflow.com/questions/13318853/opencv-drawmatches-queryidx-and-trainidx/13320083#13320083
      VectorOfVectorOfDMatch nnMatches = new VectorOfVectorOfDMatch();
      VectorOfPointF itemCoords = new VectorOfPointF();
      VectorOfPointF imageCoords = new VectorOfPointF();
      int inliers;
      double inlierRatio;

      CvInvoke.CvtColor(image, image, ColorConversion.Rgb2Gray);

      this._state = FeatureState.MATCHING;

      using (AKAZE akaze = new AKAZE())
      using (BFMatcher matcher = new BFMatcher(DistanceType.Hamming))
      {
        // find keypoints and describe their scale and orientation relative 
        // to current image
        akaze.DetectAndCompute(image, null, imageKeypoints, imageDescriptors, false);
        // "find the item in the image" -> query match to train descriptor
        /*
         * finds the k best matches of query descriptors to train descriptors
         * in this case at most 2 train descriptors for each query descriptor
         */
        matcher.Add(this._DESCRIPTORS);
        matcher.KnnMatch(imageDescriptors, nnMatches, KTH_NEAREST_NEIGHBOUR, null);

        // find key points which are distinct
        /*
         * distance of first pair of query and train descriptor match
         * should be less than second pair for a specific query descriptor
         */
        for (int i = 0; i < nnMatches.Size; i++)
        {
          MDMatch first = nnMatches[i][0];
          float dist1 = nnMatches[i][0].Distance; // distance between query and train descriptor
          float dist2 = nnMatches[i][1].Distance; // distance between query and train descriptor

          if (dist1 < NN_MATCH_RATIO * dist2)
          {
            itemCoords.Push(new System.Drawing.PointF[] {
              this._KEYPOINTS[first.TrainIdx].Point
            });
            imageCoords.Push(new System.Drawing.PointF[] {
              imageKeypoints[first.QueryIdx].Point
            });
          }
        }

        // only generate homography matrix if more than 50 matches found
        if (itemCoords.Size > MATCHES_REQUIRED)
        {
          inliers = CheckHomography(itemCoords, imageCoords, out Mat homography);
          inlierRatio = inliers * 1.0 / itemCoords.Size;
          //UnityEngine.MonoBehaviour.print("matches " + itemCoords.Size + " inlier ratio " + inlierRatio);
          if (inlierRatio > INLIER_USABLE_THRESHOLD)
          {
            this._homographyMatchMat.Dispose();
            this._homographyMatchMat = homography;
            this._inliers = inliers;
            this._matches = itemCoords.Size;
            this._inlierRatio = inlierRatio;
            this._previousPoints.Clear();
            this._previousPoints.Push(imageCoords.ToArray());
            this._previousBorderPoints.Clear();
            this._previousBorderPoints.Push(BORDER.ToArray());
            this._previousFrame = Memory.Frame.Clone();
            this._replacePreviousBorderPoints = true;
          }
        }
        else
        {
          this._homographyMatchMat.Dispose();
          this._homographyMatchMat = new Mat();
          this._matches = 0;
          this._inliers = 0;
          this._inlierRatio = 0;
          this._previousPoints.Clear();
          this._previousBorderPoints.Clear();
        }
      }
    }

    public bool FindObject(bool matching = true)
    {
      if (this._inlierRatio > INLIER_USABLE_THRESHOLD && this._previousBorderPoints.Size > 0)
      {
        this._borderPoints.Clear();

        // transform item points to image points
        CvInvoke.PerspectiveTransform(this._previousBorderPoints, this._borderPoints,
          matching ? this._homographyMatchMat : this._homographyTrackMat);

        if (this._replacePreviousBorderPoints)
        {
          this._previousBorderPoints.Clear();
          this._previousBorderPoints.Push(this._borderPoints.ToArray());
          this._replacePreviousBorderPoints = false;
        }

        return true;
      }

      return false;
    }

    public bool TrackObject()
    {
      Mat previousFrame = this._previousFrame.Clone();
      Mat currentFrame = Memory.Frame.Clone();
      System.Drawing.PointF[] imagePoints = new System.Drawing.PointF[] { };
      int inliers = 0;

      this._trackerAvgErr = 0;

      if (this._previousPoints.Size > 0)
      {
        Array.Clear(this._trackerStatus, 0, this._trackerStatus.Length);
        Array.Clear(this._trackerErr, 0, this._trackerErr.Length);

        this._state = FeatureState.TRACKING;

        CvInvoke.CvtColor(previousFrame, previousFrame, ColorConversion.Rgb2Gray);
        CvInvoke.CvtColor(currentFrame, currentFrame, ColorConversion.Rgb2Gray);

        CvInvoke.CalcOpticalFlowPyrLK(
          previousFrame, currentFrame,
          this._previousPoints.ToArray(),
          SEARCH_WINDOW_SIZE, MAX_PYRAMID_LEVELS, TERMINATION_CRITERIA,
          out imagePoints, out this._trackerStatus, out this._trackerErr
        );

        foreach (float err in this._trackerErr)
          this._trackerAvgErr += err;

        this._trackerAvgErr /= this._trackerErr.Length;

        if (this._trackerAvgErr > ACCEPTABLE_TRACKING_AVERAGE_ERROR)
        {
          this._homographyTrackMat.Dispose();
          this._homographyTrackMat = new Mat();
          this.ComputeAndMatch();
        }
        else
        {
          inliers = CheckHomography(this._previousPoints, new VectorOfPointF(imagePoints), out Mat homography);

          if (inliers <= MATCHES_REQUIRED)
          {
            this._homographyTrackMat.Dispose();
            this._homographyTrackMat = new Mat();
            this.ComputeAndMatch();
          }
          else
          {
            this._homographyTrackMat.Dispose();
            this._homographyTrackMat = homography;

            return true;
          }
        }
      }
      else
      {
        this._homographyTrackMat.Dispose();
        this._homographyTrackMat = new Mat();
        this.ComputeAndMatch();
      }

      return false;
    }

    public Frame DrawObjectBorder(bool drawAxes = false, Mat cameraMat = null, Mat distCoeffs = null, Mat rotationMat = null, Mat translationVector = null)
    {
      Mat frame = Memory.Frame.Clone();

      if (this._inlierRatio > INLIER_USABLE_THRESHOLD && this._borderPoints.Size > 0)
      {
        CvInvoke.Line(frame,
          new System.Drawing.Point((int)this._borderPoints[0].X, (int)this._borderPoints[0].Y),
          new System.Drawing.Point((int)this._borderPoints[1].X, (int)this._borderPoints[1].Y),
          new Rgb(255, 0, 0).MCvScalar, 5);
        CvInvoke.Line(frame,
          new System.Drawing.Point((int)this._borderPoints[0].X, (int)this._borderPoints[0].Y),
          new System.Drawing.Point((int)this._borderPoints[2].X, (int)this._borderPoints[2].Y),
          new Rgb(255, 0, 0).MCvScalar, 5);
        CvInvoke.Line(frame,
          new System.Drawing.Point((int)this._borderPoints[2].X, (int)this._borderPoints[2].Y),
          new System.Drawing.Point((int)this._borderPoints[3].X, (int)this._borderPoints[3].Y),
          new Rgb(255, 0, 0).MCvScalar, 5);
        CvInvoke.Line(frame,
          new System.Drawing.Point((int)this._borderPoints[1].X, (int)this._borderPoints[1].Y),
          new System.Drawing.Point((int)this._borderPoints[3].X, (int)this._borderPoints[3].Y),
          new Rgb(255, 0, 0).MCvScalar, 5);

        if (drawAxes && cameraMat != null && rotationMat != null && translationVector != null)
        {
          this.DrawOrientationAxis(cameraMat, distCoeffs, rotationMat, translationVector, frame, out frame);
        }
      }

      using (Image<Bgr, byte> nextFrame = frame.ToImage<Bgr, byte>())
      {
        System.Drawing.Bitmap currentFrame = nextFrame.ToBitmap();
        MemoryStream m = new MemoryStream();
        System.Drawing.Imaging.ImageFormat format =
          !this._UNITY ? System.Drawing.Imaging.ImageFormat.Bmp : currentFrame.RawFormat;
        currentFrame.Save(m, format);

        return new Frame(currentFrame.Height, currentFrame.Width, m.ToArray());
      }
    }

    public bool GetProjectionMatrix(IInputArray cameraMat, IInputArray distCoeffs, out Mat projectionMatrix)
    {
      projectionMatrix = Mat.Eye(4, 4, DepthType.Cv64F, 3);

      if (this.GetHomography(out Mat homography))
      {
        if (this.GetPose(cameraMat, distCoeffs, out Mat rotationMat, out Mat translationVector))
        {
          double val;

          rotationMat = rotationMat.T();

          for (int i = 0; i < rotationMat.Rows; i++)
          {
            for (int j = 0; j < rotationMat.Cols; j++)
            {
              val = rotationMat.GetValue(i, j) * -1;
              projectionMatrix.SetValue(i, j, val);
            }
            val = translationVector.GetValue(i, 0);
            projectionMatrix.SetValue(i, 3, val);
          }

          projectionMatrix.Dot(cameraMat);
        }
      }

      return false;
    }


    public Matrix<double> projection_mat(Matrix<double> H, Matrix<double> Cam_Mat, out Matrix<double> r, out Matrix<double> t)
    {
      //CvInvoke.Invert(H, H, DecompMethod.LU);
      H = H * -1;
      Matrix<double> rot_trans = H;
      Matrix<double> Cam_Mat_Inv = new Matrix<double>(3, 3);
      CvInvoke.Invert(Cam_Mat, Cam_Mat_Inv, DecompMethod.LU);
      rot_trans = H * Cam_Mat_Inv;

      //System.Diagnostics.Debug.WriteLine("Size of the Homography Mat is (row x col): " + Convert.ToString(H.Rows) + " x " + Convert.ToString(H.Cols));
      //System.Diagnostics.Debug.WriteLine("Size of the rot_trans matrix is (row x col): " + Convert.ToString(rot_trans.Rows) + " x " + Convert.ToString(rot_trans.Cols));

      Matrix<double> col1 = rot_trans.GetCol(0);
      Matrix<double> col2 = rot_trans.GetCol(1);
      Matrix<double> col3 = rot_trans.GetCol(2);

      //System.Diagnostics.Debug.WriteLine("Size of the col is (row x col): " + Convert.ToString(col1.Rows) + " x " + Convert.ToString(col1.Cols));

      //normalizing vectors
      double l = Math.Sqrt(CvInvoke.Norm(col1) * CvInvoke.Norm(col2));
      //double l = Math.Sqrt(rot_trans[0, 0] * rot_trans[0, 0] + rot_trans[1, 0] * rot_trans[1, 0] + rot_trans[2, 0] * rot_trans[2, 0]);

      //System.Diagnostics.Debug.WriteLine("Value of l: " + Convert.ToString(l));

      Matrix<double> rot_1 = col1 / l;
      Matrix<double> rot_2 = col2 / l;
      Matrix<double> tr = col3 / l;

      //calculating the othogonal base
      Matrix<double> c = rot_1 + rot_2;
      Matrix<double> p = CrossProduct(rot_1, rot_2);
      Matrix<double> d = CrossProduct(c, p);
      //System.Diagnostics.Debug.WriteLine("Size of the c matrix is (row x col): " + Convert.ToString(c.Rows) + " x " + Convert.ToString(c.Cols));
      //System.Diagnostics.Debug.WriteLine("Size of the p matrix is (row x col): " + Convert.ToString(p.Rows) + " x " + Convert.ToString(p.Cols));
      //System.Diagnostics.Debug.WriteLine("Size of the d matrix is (row x col): " + Convert.ToString(d.Rows) + " x " + Convert.ToString(d.Cols));

      rot_1 = (c / CvInvoke.Norm(c) + d / CvInvoke.Norm(d)) / Math.Sqrt(2);
      rot_2 = (c / CvInvoke.Norm(c) - d / CvInvoke.Norm(d)) / Math.Sqrt(2);

      Matrix<double> rot_3 = CrossProduct(rot_1, rot_2);
      //System.Diagnostics.Debug.WriteLine("Size of the rot matrix is (row x col): " + Convert.ToString(rot_1.Rows) + " x " + Convert.ToString(rot_1.Cols));

      Matrix<double> extrinsics = rot_1
        .ConcateHorizontal(rot_2)
        .ConcateHorizontal(rot_3)
        .ConcateHorizontal(tr);
      //System.Diagnostics.Debug.WriteLine("Size of the rotation(pre-4_4_conv) matrix is (row x col): " + Convert.ToString(projection_mat.Rows) + " x " + Convert.ToString(projection_mat.Cols));

      r = rot_1
        .ConcateHorizontal(rot_2)
        .ConcateHorizontal(rot_3);
      t = tr.Clone();

      Matrix<double> projection_mat = Cam_Mat * extrinsics;
      projection_mat = ConvertTo4_4(projection_mat); // adding the [0 0 0 1] to the last row to convert into a 4x4 for Unity's Dimension
      projection_mat = ConvertToLHS(projection_mat);   // convert to the LHS for Unity
      return projection_mat;
    }

    private Matrix<double> ConvertTo4_4(Matrix<double> a)
    {
      var bott_row = new Matrix<double>(1, 4);

      bott_row[0, 0] = 0;
      bott_row[0, 1] = 0;
      bott_row[0, 2] = 0;
      bott_row[0, 3] = 1;
      //System.Diagnostics.Debug.WriteLine("Size of the convert4x4 matrix is (row x col): " + Convert.ToString(bott_row.Rows) + " x " + Convert.ToString(bott_row.Cols));

      a = a.ConcateVertical(bott_row);
      //UnityEngine.MonoBehaviour.print("convert to 4 by 4 " + a.Rows + " " + a.Cols);
      return a;
    }

    public Matrix<double> ConvertToLHS(Matrix<double> rot_mat)
    {
      var LHSflipBackMatrix = new Matrix<double>(4, 4);
      LHSflipBackMatrix.SetIdentity();
      //LHSflipBackMatrix[0, 0] = -1.0;
      //LHSflipBackMatrix[1, 1] = -1.0;
      //LHSflipBackMatrix[2, 2] = -1.0;
      //UnityEngine.MonoBehaviour.print("convert from r to l " + rot_mat.Rows + " " + rot_mat.Cols);
      LHSflipBackMatrix = rot_mat * LHSflipBackMatrix;

      rot_mat = LHSflipBackMatrix.Clone();
      rot_mat[0, 3] *= -1;
      rot_mat[1, 3] *= -1;
      rot_mat[2, 3] *= -1;
      //rot_mat[1, 3] = LHSflipBackMatrix[2, 3];
      //rot_mat[2, 3] = LHSflipBackMatrix[1, 3];

      return rot_mat;
    }


    public Matrix<double> CrossProduct(Matrix<double> a, Matrix<double> b)
    {

      //cx = aybz − azby
      double c_x = (a[1, 0] * b[2, 0]) - (a[2, 0] * b[1, 0]);

      //cy = azbx − axbz
      double c_y = (a[2, 0] * b[0, 0]) - (a[0, 0] * b[2, 0]);

      //cz = axby − aybx
      double c_z = (a[0, 0] * b[1, 0]) - (a[1, 0] * b[0, 0]);

      //System.Diagnostics.Debug.WriteLine("c_x: " + Convert.ToString(c_x));
      //System.Diagnostics.Debug.WriteLine("c_y: " + Convert.ToString(c_y));
      //System.Diagnostics.Debug.WriteLine("c_z: " + Convert.ToString(c_z));
      var res = new Matrix<double>(3, 1);
      res[0, 0] = c_x;
      res[1, 0] = c_y;
      res[2, 0] = c_z;


      return res;

    }

    public bool GetPose(IInputArray cameraMat, IInputArray distCoeffs, out Mat rotationMat, out Mat translationVector)
    {
      VectorOfPoint3D32F objectCoords = new VectorOfPoint3D32F(new MCvPoint3D32f[]
      {
        new MCvPoint3D32f(this.BORDER[1].X/2, this.BORDER[2].Y/2, 1), // center
        new MCvPoint3D32f(this.BORDER[0].X, this.BORDER[2].Y/2, 1), // x-axis
        new MCvPoint3D32f(this.BORDER[1].X/2, this.BORDER[0].Y, 1), // y-axis
        //new MCvPoint3D32f(this.BORDER[1].X/2, this.BORDER[2].Y/2, -150), // z-axis
        new MCvPoint3D32f(this.BORDER[0].X, this.BORDER[0].Y, 1),
        new MCvPoint3D32f(this.BORDER[1].X, this.BORDER[1].Y, 1),
        new MCvPoint3D32f(this.BORDER[2].X, this.BORDER[2].Y, 1),
        new MCvPoint3D32f(this.BORDER[3].X, this.BORDER[3].Y, 1),
      });
      VectorOfPointF imageCoords = new VectorOfPointF();
      Matrix<double> homography = new Matrix<double>(3, 3);
      Matrix<double> temp;
      Matrix<double> pointMat = new Matrix<double>(3, 1);
      bool foundPose = false;

      // axes = new VectorOfPointF();
      rotationMat = new Mat();
      translationVector = new Mat();

      if (!this._homographyMatchMat.IsEmpty)
      {
        this._homographyMatchMat.CopyTo(homography);

        if (!this._homographyTrackMat.IsEmpty)
        {
          temp = new Matrix<double>(3, 3);
          this._homographyTrackMat.CopyTo(temp);
          (temp * homography).CopyTo(homography);
        }

        foreach (MCvPoint3D32f point in objectCoords.ToArray())
        {
          pointMat[0, 0] = point.X;
          pointMat[1, 0] = point.Y;
          pointMat[2, 0] = point.Z;

          (homography * pointMat).Mat.ConvertTo(pointMat, DepthType.Cv64F);

          imageCoords.Push(new System.Drawing.PointF[]
          {
            new System.Drawing.PointF((float) (pointMat[0,0]/pointMat[2,0]), (float) (pointMat[1,0]/pointMat[2,0]))
          });
        }

        if (distCoeffs != null)
        {
          foundPose = CvInvoke.SolvePnP(objectCoords, imageCoords,
            cameraMat, distCoeffs, this._rotationMatrix, this._translationVector,
            this._useExtrinsicGuessForPnP, SolvePnpMethod.Iterative);
        }
        else
        {
          foundPose = CvInvoke.SolvePnP(objectCoords, imageCoords,
            cameraMat, Mat.Zeros(4, 1, DepthType.Cv64F, 1), this._rotationMatrix, this._translationVector,
            this._useExtrinsicGuessForPnP, SolvePnpMethod.Iterative);
        }

        if (this._useExtrinsicGuessForPnP == false)
          this._useExtrinsicGuessForPnP = true;

        /*
        axes.Push(new System.Drawing.PointF[] {
          imageCoords[0], imageCoords[1], imageCoords[2]//, imageCoords[3]
        });
        */

        this._rotationMatrix.ConvertTo(rotationMat, DepthType.Cv32F);
        this._translationVector.ConvertTo(translationVector, DepthType.Cv32F);
      }

      return foundPose;
    }

    public bool GetCenter(IInputArray cameraMat, IInputArray distCoeffs, out Mat rotationMat, out Mat translationVector, out float[] center)
    {
      center = new float[3];

      if (GetPose(cameraMat, distCoeffs, out rotationMat, out translationVector))
      {
        System.Drawing.PointF[] imageCoord = CvInvoke.ProjectPoints(new MCvPoint3D32f[] { new MCvPoint3D32f(this.BORDER[1].X / 2, this.BORDER[2].Y / 2, 1) },
          rotationMat, translationVector, cameraMat, distCoeffs);

        center[0] = imageCoord[0].X;
        center[1] = imageCoord[0].Y;
        center[2] = 1;

        return true;
      }

      return false;
    }

    // refer to https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToEuler/index.htm
    public float[] GetEulerAngles(Mat rotationMat)
    {
      Mat r = new Mat();

      if ((rotationMat.Size.Height == 1 && rotationMat.Size.Width == 3)
        || (rotationMat.Size.Height == 3 && rotationMat.Size.Width == 1))
      {
        CvInvoke.Rodrigues(rotationMat, r);
      }
      else
        rotationMat.ConvertTo(r, DepthType.Cv32F);

      float[] eulerAngles = new float[3];
      double bank, altitude, heading;

      // assume that angles are in radians
      if (r.GetValue(1, 0) > 0.998)
      { // singularity at north pole
        bank = 0;
        altitude = Math.PI / 2;
        heading = Math.Atan2(r.GetValue(0, 2), r.GetValue(2, 2));
      }
      else if (r.GetValue(1, 0) < -0.998)
      { // singularity at south pole
        bank = 0;
        altitude = -Math.PI / 2;
        heading = Math.Atan2(r.GetValue(0, 2), r.GetValue(2, 2));
      }
      else
      {
        bank = Math.Atan2(-r.GetValue(1, 2), r.GetValue(1, 1));
        altitude = Math.Asin(r.GetValue(1, 0));
        heading = Math.Atan2(-r.GetValue(2, 0), r.GetValue(0, 0));
      }

      eulerAngles[0] = (float)bank; // rotation about x
      eulerAngles[1] = (float)heading; // rotation about y 
      eulerAngles[2] = (float)altitude; // rotation about z

      return eulerAngles;
    }

    public void DrawOrientationAxis(Mat cameraMat, Mat distCoeffs, Mat rotationMat, Mat translationVector, Mat srcFrame, out Mat dstFrame)
    {
      VectorOfPoint3D32F objectCoords = new VectorOfPoint3D32F(new MCvPoint3D32f[]
      {
        new MCvPoint3D32f(this.BORDER[1].X/2, this.BORDER[2].Y/2, 1), // center
        new MCvPoint3D32f(this.BORDER[1].X, this.BORDER[2].Y/2, 1), // x-axis
        new MCvPoint3D32f(this.BORDER[1].X/2, this.BORDER[1].Y, 1), // y-axis
        new MCvPoint3D32f(this.BORDER[1].X/2, this.BORDER[2].Y/2, -150), // z-axis
        new MCvPoint3D32f(this.BORDER[0].X, this.BORDER[0].Y, 1),
        new MCvPoint3D32f(this.BORDER[1].X, this.BORDER[1].Y, 1),
        new MCvPoint3D32f(this.BORDER[2].X, this.BORDER[2].Y, 1),
        new MCvPoint3D32f(this.BORDER[3].X, this.BORDER[3].Y, 1),
      });

      dstFrame = srcFrame.Clone();
      System.Drawing.PointF[] imageCoords;

      if (distCoeffs != null)
        imageCoords = CvInvoke.ProjectPoints(objectCoords.ToArray(), rotationMat, translationVector, cameraMat, distCoeffs);
      else
        imageCoords = CvInvoke.ProjectPoints(objectCoords.ToArray(), rotationMat, translationVector, cameraMat, Mat.Zeros(4, 1, DepthType.Cv64F, 1));


      CvInvoke.Line(dstFrame,
        new System.Drawing.Point((int)imageCoords[4].X, (int)imageCoords[4].Y),
        new System.Drawing.Point((int)imageCoords[5].X, (int)imageCoords[5].Y),
        new Rgb(100, 100, 100).MCvScalar, 5);
      CvInvoke.Line(dstFrame,
        new System.Drawing.Point((int)imageCoords[4].X, (int)imageCoords[4].Y),
        new System.Drawing.Point((int)imageCoords[6].X, (int)imageCoords[6].Y),
        new Rgb(100, 100, 100).MCvScalar, 5);
      CvInvoke.Line(dstFrame,
        new System.Drawing.Point((int)imageCoords[6].X, (int)imageCoords[6].Y),
        new System.Drawing.Point((int)imageCoords[7].X, (int)imageCoords[7].Y),
        new Rgb(100, 100, 100).MCvScalar, 5);
      CvInvoke.Line(dstFrame,
        new System.Drawing.Point((int)imageCoords[5].X, (int)imageCoords[5].Y),
        new System.Drawing.Point((int)imageCoords[7].X, (int)imageCoords[7].Y),
        new Rgb(100, 100, 100).MCvScalar, 5);
      CvInvoke.Line(dstFrame,
        new System.Drawing.Point((int)imageCoords[0].X, (int)imageCoords[0].Y),
        new System.Drawing.Point((int)imageCoords[1].X, (int)imageCoords[1].Y),
        new Rgb(0, 0, 255).MCvScalar, 10); // red - x (x in unity)
      CvInvoke.Line(dstFrame,
        new System.Drawing.Point((int)imageCoords[0].X, (int)imageCoords[0].Y),
        new System.Drawing.Point((int)imageCoords[2].X, (int)imageCoords[2].Y),
        new Rgb(255, 0, 0).MCvScalar, 10); // blue - y (z in unity)
      CvInvoke.Line(dstFrame,
        new System.Drawing.Point((int)imageCoords[0].X, (int)imageCoords[0].Y),
        new System.Drawing.Point((int)imageCoords[3].X, (int)imageCoords[3].Y),
        new Rgb(0, 255, 0).MCvScalar, 10); // green - z (y in unity)
    }
  }

  public class InitialFrame
  {
    private readonly MCvTermCriteria TERMINATION_CRITERIA
      = new MCvTermCriteria(30, double.Epsilon);
    private const float RANSAC_REPROJECTION_THRESHOLD = 3f; // default

    private readonly Camera _cap;
    private readonly System.Drawing.Size _patternSize;
    private readonly float _squareSize;
    private bool _calibrated;
    private Mat _cameraMat, _distCoeffs;
    private VectorOfMat _rotationVectors, _translationVector;
    // private VectorOfVectorOfPointF _imageCoords;
    private double _err;

    public InitialFrame(Camera cap, Size patternSize, float squareSize)
    {
      this._cap = cap;
      this._patternSize = patternSize != null ? patternSize.Dims : new System.Drawing.Size(0, 0);
      this._squareSize = squareSize;
      this._calibrated = false;
      this._cameraMat = Mat.Eye(3, 3, DepthType.Cv32F, 1);
      this._distCoeffs = Mat.Zeros(5, 1, DepthType.Cv64F, 1);
      this._rotationVectors = new VectorOfMat();
      this._translationVector = new VectorOfMat();
      // this._imageCoords = new VectorOfVectorOfPointF();
    }

    public InitialFrame() : this(null, null, -1) { }

    // TODO convert to non-opencv data types
    public Mat CameraMatrix { get => this._cameraMat; }
    public Mat DistortionCoefficients { get => this._distCoeffs; }
    public VectorOfMat RotationVectors { get => this._rotationVectors; }
    public VectorOfMat TranslationVector { get => this._translationVector; }
    public bool IsCalibrated { get => this._calibrated; }
    public double Error { get => this._err; }

    public static VectorOfPoint3D32F CalcCenters(System.Drawing.Size boardSize, float squareSize)
    {
      List<MCvPoint3D32f> centers = new List<MCvPoint3D32f>();

      for (int i = 0; i < boardSize.Height; i++)
        for (int j = 0; j < boardSize.Width; j++)
          centers.Add(new MCvPoint3D32f((float)j * squareSize, (float)i * squareSize, 0));

      return new VectorOfPoint3D32F(centers.ToArray());
    }

    public void Start()
    {
      if (this._cap == null) return;

      bool patternFound;
      int nframes = 0;
      VectorOfPoint3D32F objectCoord = CalcCenters(this._patternSize, this._squareSize);
      VectorOfPointF centers = new VectorOfPointF();
      VectorOfVectorOfPoint3D32F objectCoords = new VectorOfVectorOfPoint3D32F();
      VectorOfVectorOfPointF imageCoords = new VectorOfVectorOfPointF();

      while (nframes < 20)
      {
        this._cap.GetNextFrame();
        Mat f = Memory.Frame.Clone();
        patternFound = CvInvoke.FindChessboardCorners(f, this._patternSize, centers);

        if (patternFound)
        {
          // this._imageCoords.Push(centers);
          imageCoords.Push(centers);
          objectCoords.Push(objectCoord);
          nframes++;
        }
      }

      this._err = CvInvoke.CalibrateCamera(objectCoords.ToArrayOfArray(),
        imageCoords.ToArrayOfArray(), Memory.Frame.Size, this._cameraMat,
        this._distCoeffs, CalibType.Default, TERMINATION_CRITERIA, out Mat[] rvec, out Mat[] tvec);

      this._rotationVectors.Push(rvec);
      this._translationVector.Push(tvec);

      /*
      this.SaveToFile(err, this._cameraMat, this._distCoeffs, this._rotationVectors,
      this._translationVector, objectCoord, this._imageCoords);
      */

      double fx, fy, cx, cy;

      this.SaveToFile(this._err, this._cameraMat, this._distCoeffs);

      this._calibrated = true;
    }

    public void SaveToFile(double err, Mat camMat, Mat distCoeffs)
    /*public void SaveToFile(double err, Mat camMat, Mat distCoeffs, VectorOfMat rvec,
      VectorOfMat tvec, VectorOfPoint3D32F objCoords, VectorOfVectorOfPointF imageCoords)*/
    {
      using (FileStorage fs =
        new FileStorage("intrinsics.yml", FileStorage.Mode.Write | FileStorage.Mode.FormatYaml))
      {
        fs.Write(err, "error");
        fs.Write(camMat, "cameraMatrix");
        fs.Write(distCoeffs, "distortionCoefficients");
        /*
         * TODO may need to store these for use in future
         * cannot store them as is right now because fs.Write only supports up to Mat
         * and nothing else (VectorOfMat, VectorOfVectorOfPoint3D32F, VectorOfVectorOfPoint...
         */
        // fs.Write(rvec, "rotationVectors");
        // fs.Write(tvec, "translationVector");
        // fs.Write(objCoords, "objectCoordinates");
        // fs.Write(imageCoords, "imageCoordinates");
        fs.ReleaseAndGetString();
      }
    }

    public void ReadFromFile()
    {
      using (FileStorage fs =
        new FileStorage("intrinsics.yml", FileStorage.Mode.Read | FileStorage.Mode.FormatYaml))
      {
        this._err = fs.GetNode("error").ReadDouble();
        fs.GetNode("cameraMatrix").ReadMat(this._cameraMat);
        fs.GetNode("distortionCoefficients").ReadMat(this._distCoeffs);
        fs.ReleaseAndGetString();
      }

      this._calibrated = true;
    }
  }

  public class ChessboardDemo
  {
    private readonly bool _unity;
    private readonly System.Drawing.Size _patternSize;
    private VectorOfPointF _centers = new VectorOfPointF();

    public ChessboardDemo(bool unity = true)
    {
      this._unity = unity;
    }

    public ChessboardDemo(Size patternSize, bool unity = true) : this(unity)
    {
      this._patternSize = patternSize.Dims;
    }

    public Frame RunDemo()
    {
      Mat f = Memory.Frame.Clone();
      bool patternFound = CvInvoke.FindChessboardCorners(f, this._patternSize, this._centers);

      CvInvoke.DrawChessboardCorners(f, this._patternSize, this._centers, patternFound);

      System.Drawing.Bitmap currentFrame = f.ToImage<Bgr, byte>().ToBitmap();
      MemoryStream m = new MemoryStream();
      System.Drawing.Imaging.ImageFormat format =
          !this._unity ? System.Drawing.Imaging.ImageFormat.Bmp : currentFrame.RawFormat;
      currentFrame.Save(m, format);

      return new Frame(currentFrame.Height, currentFrame.Width, m.ToArray());
    }
  }

  public class Test
  {
    // taken from http://www.emgu.com/wiki/index.php/Hello_World_in_CSharp
    public static void HelloWord()
    {
      String win1 = "Hello World"; //The name of the window
      CvInvoke.NamedWindow(win1); //Create the window using the specific name

      Mat img = new Mat(200, 600, DepthType.Cv8U, 3); //Create a 3 channel image of 400x200
      img.SetTo(new Bgr(255, 0, 0).MCvScalar); // set it to Blue color

      //Draw "Hello, world." on the image using the specific font
      CvInvoke.PutText(
         img,
         "Hello, world! Press any key to continue",
         new System.Drawing.Point(10, 80),
         FontFace.HersheyComplex,
         0.5,
         new Bgr(0, 255, 0).MCvScalar);

      CvInvoke.Imshow(win1, img); //Show the image
      CvInvoke.WaitKey(0);  //Wait for the key pressing event
      CvInvoke.DestroyWindow(win1); //Destroy the window if key is pressed
    }

    public static void CaptureAndShow()
    {
      Camera capture = new Camera(new Size(1440, 2560), false);
      Frame frame;
      Image<Bgr, byte> img;

      String win1 = "Camera Demo";
      CvInvoke.NamedWindow(win1, NamedWindowType.Normal);

      for (; ; )
      {
        frame = capture.GetNextFrame();
        img = new Image<Bgr, byte>(frame.Width, frame.Height)
        {
          Bytes = frame.Image
        };

        CvInvoke.Flip(img, img, FlipType.Vertical);
        CvInvoke.Imshow(win1, img);
        if (27 == CvInvoke.WaitKey(100)) break; // 27 is ESC key
      }

      CvInvoke.DestroyWindow(win1);
    }

    public static void RunChessboardDemo(Size patternSize)
    {
      Camera capture = new Camera(0, unity: false);
      Frame frame;
      Image<Bgr, byte> img;

      String win1 = "Chessboard Demo";
      CvInvoke.NamedWindow(win1);

      for (; ; )
      {
        capture.GetNextFrame();
        frame = (new ChessboardDemo(patternSize, false)).RunDemo();
        img = new Image<Bgr, byte>(frame.Width, frame.Height)
        {
          Bytes = frame.Image
        };

        /*CvInvoke.PutText(
          img.Mat,
          "Hello, world",
          new System.Drawing.Point(10, 10),
          FontFace.HersheyComplex,
          1.0,
          new Bgr(0, 255, 0).MCvScalar,
          1,
          LineType.EightConnected,
          true);*/

        CvInvoke.Imshow(win1, img);
        if (27 == CvInvoke.WaitKey(100)) break; // 27 is ESC key
      }

      CvInvoke.DestroyWindow(win1);
    }

    public static void CalibrateCamera(Size patternSize, float squareSize)
    {
      Camera capture = new Camera(0, unity: false);
      Image<Bgr, byte> img;

      String win1 = "Camera Calibration Demo";
      CvInvoke.NamedWindow(win1);

      InitialFrame cc = new InitialFrame(capture, patternSize, squareSize);
      cc.Start();

      for (; ; )
      {
        capture.GetNextFrame();
        img = Memory.Frame.ToImage<Bgr, byte>();

        CvInvoke.PutText(
          img.Mat,
          cc.IsCalibrated ? "Calibrated" : "Calibrating",
          new System.Drawing.Point(10, 10),
          FontFace.HersheyComplex,
          1.0,
          new Bgr(0, 255, 0).MCvScalar,
          1,
          LineType.EightConnected,
          true);

        CvInvoke.Imshow(win1, img);
        if (27 == CvInvoke.WaitKey(100)) break; // 27 is ESC key
      }
    }

    public static void TestProjectionMatrix(string imageFilePath, string keypointsFilePath, string image2FilePath, string image3FilePath)
    {
      Size size = new Size(720, 1080);
      System.Diagnostics.Debug.WriteLine("Starting Program...");

      // getting key points of original image
      FeaturePoints.ComputeAndSave(imageFilePath, keypointsFilePath);
      FeaturePoints fp = FeaturePoints.ReadData(keypointsFilePath, unity: false);

      Camera capture = new Camera(0, size, false);

      // Getting intrisic files
      InitialFrame ip = new InitialFrame(capture, new Size(4, 7), 30);
      System.Diagnostics.Debug.WriteLine("Going for Intrinsics...");
      //ip.Start();
      ip.ReadFromFile();
      System.Diagnostics.Debug.WriteLine("Got Intrinsics");

      //setting the matched image
      Memory.Frame = CvInvoke.Imread(image2FilePath);

      // trying to detect matched image
      fp.ComputeAndMatch();
      System.Diagnostics.Debug.WriteLine("Trying to  Match");

      if (fp.FindObject())
      {
        String win1 = "Matching - Projection Matrix Test";
        //CvInvoke.NamedWindow(win1, NamedWindowType.Normal);

        //call to projection matrix
        if (fp.GetHomography(out Emgu.CV.Mat H))
        {
          Emgu.CV.Matrix<double> H_mat = new Emgu.CV.Matrix<double>(3, 3);
          Emgu.CV.Matrix<double> cam_mat = new Emgu.CV.Matrix<double>(3, 3);
          for (int i = 0; i < 3; i++)
          {
            for (int j = 0; j < 3; j++)
            {
              double val = ARKit.MatExtension.GetValue(H, i, j);
              //print("i: " + i.ToString());
              //print("j: " + j.ToString());
              H_mat[i, j] = val;

              val = ARKit.MatExtension.GetValue(ip.CameraMatrix, i, j);
              cam_mat[i, j] = val;
            }
          }
          //Emgu.CV.Matrix<double> proj_match = fp.projection_mat(H_mat, cam_mat);
        }
      }


      System.Diagnostics.Debug.WriteLine("Finished Matching");

      //Tracking Begins

      //setting the memory frame to third image
      Memory.Frame = CvInvoke.Imread(image3FilePath);

      fp.TrackObject();

      if (fp.FindObject(false))
      {
        System.Diagnostics.Debug.WriteLine("Started Tracking");
        // String win2 = "Tracking - Projection Matrix Test";
        //CvInvoke.NamedWindow(win2, NamedWindowType.Normal);

        //call to projection matrix
        if (fp.GetHomography(out Emgu.CV.Mat H_track))
        {
          Emgu.CV.Matrix<double> H_mat = new Emgu.CV.Matrix<double>(3, 3);
          Emgu.CV.Matrix<double> cam_mat = new Emgu.CV.Matrix<double>(3, 3);
          for (int i = 0; i < 3; i++)
          {
            for (int j = 0; j < 3; j++)
            {
              double val = ARKit.MatExtension.GetValue(H_track, i, j);
              //print("i: " + i.ToString());
              //print("j: " + j.ToString());
              H_mat[i, j] = val;

              val = ARKit.MatExtension.GetValue(ip.CameraMatrix, i, j);
              cam_mat[i, j] = val;
            }
          }
          //Emgu.CV.Matrix<double> proj_track = fp.projection_mat(H_mat, cam_mat);

        }

      }

      System.Diagnostics.Debug.WriteLine("Finished Tracking & Matching");

    }





    public static void ReadIntrinsicsFile()
    {
      InitialFrame cc = new InitialFrame();
      cc.ReadFromFile();
      System.Diagnostics.Debug.WriteLine(cc.IsCalibrated);
      System.Diagnostics.Debug.WriteLine(cc.Error);

      /*
       * refer to https://stackoverflow.com/questions/32255440/how-can-i-get-and-set-pixel-values-of-an-emgucv-mat-image
       * for accessing elements in a matrix
       */
      System.Diagnostics.Debug.WriteLine(((Object)cc.CameraMatrix.GetValue(0, 0)).ToString());
      System.Diagnostics.Debug.WriteLine(((Object)cc.DistortionCoefficients.GetValue(0, 0)).ToString());
    }

    public static void GetFeaturePoints(string imageFilePath, string keypointsFilePath)
    {
      FeaturePoints.ComputeAndSave(imageFilePath, keypointsFilePath);
    }

    public static void ReadFeaturePoints(string keypointsFilePath)
    {
      FeaturePoints fp = FeaturePoints.ReadData(keypointsFilePath, unity: false);
      MKeyPoint kp = fp.KeyPoints.ToArray()[0];

      System.Diagnostics.Debug.WriteLine(kp.Angle);
      System.Diagnostics.Debug.WriteLine(kp.ClassId);
      System.Diagnostics.Debug.WriteLine(kp.Octave);
      System.Diagnostics.Debug.WriteLine(kp.Point.X);
      System.Diagnostics.Debug.WriteLine(kp.Point.Y);
      System.Diagnostics.Debug.WriteLine(kp.Response);
      System.Diagnostics.Debug.WriteLine(kp.Size);
      System.Diagnostics.Debug.WriteLine(((Object)fp.Descriptors.GetValue(0, 0)).ToString());
    }

    public static void MatchFeatures(string imageFilePath, string keypointsFilePath, Size size)
    {
      FeaturePoints.ComputeAndSave(imageFilePath, keypointsFilePath);
      FeaturePoints fp = FeaturePoints.ReadData(keypointsFilePath, unity: false);

      Camera capture = new Camera(0, size, false);
      Frame frame;
      Image<Bgr, byte> img;

      String win1 = "Feature Matching Demo";
      CvInvoke.NamedWindow(win1, NamedWindowType.Normal);

      for (; ; )
      {
        capture.GetNextFrame();
        fp.ComputeAndMatch();
        fp.FindObject();
        frame = fp.DrawObjectBorder();
        img = new Image<Bgr, byte>(frame.Width, frame.Height)
        {
          Bytes = frame.Image
        };

        System.Diagnostics.Debug.WriteLine("Matches " + fp.Matches);
        System.Diagnostics.Debug.WriteLine("Inliers " + fp.Inliers);
        System.Diagnostics.Debug.WriteLine("Inlier Ratio " + fp.InlierRatio);

        CvInvoke.Flip(img, img, FlipType.Vertical);
        CvInvoke.Imshow(win1, img);
        if (27 == CvInvoke.WaitKey(100)) break; // 27 is ESC key
      }

      CvInvoke.DestroyWindow(win1);
    }

    public static void TrackFeatures(string imageFilePath, string keypointsFilePath, Size size)
    {
      InitialFrame ip;
      FeaturePoints.ComputeAndSave(imageFilePath, keypointsFilePath);
      FeaturePoints fp;
      int ntracks = 0, nmatches = 0;
      bool tracked = false;

      Camera capture = new Camera(1, size, false);
      Frame frame;
      Image<Bgr, byte> img;

      String win1 = "Feature Tracking Demo";
      CvInvoke.NamedWindow(win1, NamedWindowType.Normal);

      if (File.Exists("intrinsics.yml"))
      {
        ip = new InitialFrame();
        ip.ReadFromFile();
      }
      else
      {
        ip = new InitialFrame(capture, new Size(4, 7), 30);
        ip.Start();
      }

      fp = FeaturePoints.ReadData(keypointsFilePath, false);

      for (int i = 0; ; i++)
      {
        capture.GetNextFrame();

        if (i == 0)
          fp.ComputeAndMatch();
        else
          tracked = fp.TrackObject();

        fp.FindObject(!tracked);

        if (fp.GetPose(ip.CameraMatrix, ip.DistortionCoefficients, out Mat rotation, out Mat translation))
        {
          frame = fp.DrawObjectBorder(true, ip.CameraMatrix, ip.DistortionCoefficients, rotation, translation);
          float[] r = fp.GetEulerAngles(rotation);
          System.Diagnostics.Debug.WriteLine("r" + r[0] + "\t" + r[1] + "\t" + r[2]);
        }
        else
          frame = fp.DrawObjectBorder();

        img = new Image<Bgr, byte>(frame.Width, frame.Height)
        {
          Bytes = frame.Image
        };

        if (fp.State == FeaturePoints.FeatureState.MATCHING && fp.Matches > 0)
        {
          nmatches++;
          System.Diagnostics.Debug.Write("Detecting, Describing, and Matching");
          System.Diagnostics.Debug.Write("\t\tPoints " + fp.KeyPoints.Size);
          System.Diagnostics.Debug.Write("\t\tMatches " + fp.Matches);
          System.Diagnostics.Debug.Write("\t\tInliers " + fp.Inliers);
          System.Diagnostics.Debug.WriteLine("\t\tInlier Ratio " + fp.InlierRatio);
        }
        else if (fp.State == FeaturePoints.FeatureState.TRACKING)
        {
          ntracks++;
          System.Diagnostics.Debug.Write("Tracking");
          System.Diagnostics.Debug.WriteLine("\t\tAverage Error " + fp.TrackerAverageError);
        }
        else
        {
          System.Diagnostics.Debug.WriteLine("Nothing happening...");
          System.Diagnostics.Debug.Write("\t\tPoints " + fp.KeyPoints.Size);
          System.Diagnostics.Debug.Write("\t\tMatches " + fp.Matches);
          System.Diagnostics.Debug.Write("\t\tInliers " + fp.Inliers);
          System.Diagnostics.Debug.WriteLine("\t\tInlier Ratio " + fp.InlierRatio);
        }

        CvInvoke.Flip(img, img, FlipType.Vertical);
        CvInvoke.Imshow(win1, img);
        if (27 == CvInvoke.WaitKey(100)) break; // 27 is ESC key
      }

      CvInvoke.DestroyWindow(win1);

      System.Diagnostics.Debug.WriteLine("Frames spent matching: " + nmatches);
      System.Diagnostics.Debug.WriteLine("Frames spent tracking: " + ntracks);
    }
  }
}
