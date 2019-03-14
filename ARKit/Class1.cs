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

  public class Size
  {
    private System.Drawing.Size _size;

    public Size(int height, int width)
    {
      this._size = new System.Drawing.Size(height, width);
    }

    public System.Drawing.Size Dims => this._size;
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

  // based on 2nd answer https://answers.unity.com/questions/52368/emgucv-inside-unity.html
  public class Camera
  {
    private readonly VideoCapture cap;
    private readonly bool unity;

    public Camera(int cameraId = 0, Size size = null, bool unity = true)
    {
      this.cap = new VideoCapture(cameraId);

      if (size.Dims.Height != 0 && size.Dims.Width != 0)
      {
        this.cap.SetCaptureProperty(CapProp.FrameHeight, size.Dims.Height);
        this.cap.SetCaptureProperty(CapProp.FrameWidth, size.Dims.Width);
      }

      this.unity = unity;
    }

    public Camera(int cameraId = 0, bool unity = true) : this(cameraId, null, unity) { }
    public Camera(Size size, bool unity = true) : this(0, size, unity) { }
    public Camera(bool unity) : this(0, unity) { }

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
    private const float INLIER_THRESHOLD = 2.5f;
    private const float INLIER_USABLE_THRESHOLD = 0.05f;
    private const int KTH_NEAREST_NEIGHBOUR = 2;
    private const int MAX_PYRAMID_LEVELS = 10; // default used in OpenCV
    private const int MATCHES_REQUIRED = 50;
    private const float NN_MATCH_RATIO = 0.8f;
    // either finish by 30 iterations or search window moved less than epsilon of 0.01
    private readonly MCvTermCriteria TERMINATION_CRITERIA =
      new MCvTermCriteria(30, 0.01); // default used in OpenCV
    private readonly System.Drawing.Size SEARCH_WINDOW_SIZE =
      new System.Drawing.Size(21, 21); // default used in OpenCV
    private const float ACCEPTABLE_TRACKING_AVERAGE_ERROR = 2.0f;
    public enum FeatureState { MATCHING, TRACKING };

    private readonly bool _unity;
    private readonly System.Drawing.Size _size;
    private readonly VectorOfKeyPoint _keypoints;
    private Mat _homographyMat;
    private readonly Mat _descriptors;
    private int _matches;
    private int _inliers;
    private double _inlierRatio;
    private VectorOfPointF _imagePoints;
    private Mat _previousFrame;
    private VectorOfPointF _previousPoints;
    private byte[] _trackerStatus;
    private Single[] _trackerErr;
    private float _trackerAvgErr;
    private FeatureState _state;

    public FeaturePoints(System.Drawing.Size size, VectorOfKeyPoint keypoints,
      Mat descriptors, bool unity = true)
    {
      this._unity = unity;
      this._size = size;
      this._keypoints = keypoints;
      this._descriptors = descriptors;
      this._homographyMat = new Mat();
      this._matches = 0;
      this._inliers = 0;
      this._inlierRatio = 0;
      this._imagePoints = new VectorOfPointF();
      this._previousFrame = new Mat();
      this._previousPoints = new VectorOfPointF();
      this._trackerStatus = new byte[] { };
      this._trackerErr = new float[] { };
      this._trackerAvgErr = 0;
      this._state = FeatureState.MATCHING;
    }

    public System.Drawing.Size Size { get => this._size; }
    public VectorOfKeyPoint KeyPoints { get => this._keypoints; }
    public Mat Descriptors { get => this._descriptors; }
    public Mat HomographyMatrix { get => this._homographyMat; }
    public int Matches { get => this._matches; }
    public int Inliers { get => this._inliers; }
    public double InlierRatio { get => this._inlierRatio; }
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

    public void ComputeAndMatch()
    {
      VectorOfKeyPoint imageKeypoints = new VectorOfKeyPoint();
      Mat imageDescriptors = new Mat(), image = Memory.Frame.Clone();
      // DMatch type explanation
      // https://stackoverflow.com/questions/13318853/opencv-drawmatches-queryidx-and-trainidx/13320083#13320083
      VectorOfVectorOfDMatch nnMatches = new VectorOfVectorOfDMatch();
      VectorOfPointF itemCoords = new VectorOfPointF();
      VectorOfPointF imageCoords = new VectorOfPointF();
      VectorOfPointF inliers1 = new VectorOfPointF();
      VectorOfPointF inliers2 = new VectorOfPointF();
      Matrix<double> homographyMat;

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
        matcher.Add(this._descriptors);
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
              this._keypoints[first.TrainIdx].Point
            });
            imageCoords.Push(new System.Drawing.PointF[] {
              imageKeypoints[first.QueryIdx].Point
            });
          }
        }

        // only generate homography matrix if more than 50 matches found
        if (itemCoords.Size > MATCHES_REQUIRED)
        {
          // determine homography matrix
          this._homographyMat = CvInvoke.FindHomography(itemCoords, imageCoords,
            HomographyMethod.Ransac, INLIER_THRESHOLD);
          homographyMat = new Matrix<double>(this._homographyMat.Rows, this._homographyMat.Cols);
          this._homographyMat.CopyTo(homographyMat);

          this._inliers = 0;

          /*
           * check that the matches fit the homography model
           * by transforming the key points of the item and
           * comparing with the detected key points in the image
           * of where the item should be
           */
          for (int i = 0; i < itemCoords.Size; i++)
          {
            Mat col = Mat.Ones(3, 1, DepthType.Cv64F, 3);
            Matrix<double> colm = new Matrix<double>(col.Rows, col.Cols);
            col.SetValue(0, 0, itemCoords[i].X);
            col.SetValue(1, 0, itemCoords[i].Y);

            col.CopyTo(colm);
            colm = homographyMat * colm;
            colm /= colm[2, 0];

            double dist = Math.Sqrt(
              Math.Pow(colm[0, 0] - imageCoords[i].X, 2) +
              Math.Pow(colm[1, 0] - imageCoords[i].Y, 2));

            if (dist < INLIER_THRESHOLD)
              /*{
                inliers1.Push(new System.Drawing.PointF[] { itemCoords[i] });
                inliers2.Push(new System.Drawing.PointF[] { imageCoords[i] });
              }*/
              this._inliers++;
          }

          // this._inliers = inliers1.Size();
          this._matches = itemCoords.Size;
          this._inlierRatio = this._inliers * 1.0 / this._matches;
        }
        else
        {
          this._matches = 0;
          this._inliers = 0;
          this._inlierRatio = 0;
        }
      }

      this._previousFrame = Memory.Frame.Clone();
    }

    public void FindObject()
    {
      this._imagePoints.Clear();

      if (this._inlierRatio > INLIER_USABLE_THRESHOLD)
      {
        VectorOfPointF imagePlaneCoords = new VectorOfPointF(new System.Drawing.PointF[] {
          new System.Drawing.PointF(0, 0),
          new System.Drawing.PointF(this._size.Width, 0),
          new System.Drawing.PointF(0, this._size.Height),
          new System.Drawing.PointF(this._size.Width, this._size.Height),
        });

        // transform item points to image points
        CvInvoke.PerspectiveTransform(imagePlaneCoords, this._imagePoints, this._homographyMat);
      }

      this._previousPoints.Clear();
      this._previousPoints.Push(this._imagePoints.ToArray());
    }

    public void TrackObject()
    {
      Mat previousFrame = this._previousFrame.Clone();
      Mat currentFrame = Memory.Frame.Clone();
      System.Drawing.PointF[] imagePoints = new System.Drawing.PointF[] { };
      Matrix<double> homographyMat;
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
          this.ComputeAndMatch();
          this.FindObject();
        }
        else
        {
          homographyMat = new Matrix<double>(this._homographyMat.Rows, this._homographyMat.Cols);
          this._homographyMat.CopyTo(homographyMat);

          /*
           * check that the matches fit the homography model
           * by transforming the key points of the item and
           * comparing with the detected key points in the image
           * of where the item should be
           */
          for (int i = 0; i < imagePoints.Length; i++)
          {
            Mat col = Mat.Ones(3, 1, DepthType.Cv64F, 3);
            Matrix<double> colm = new Matrix<double>(imagePoints.Length, 1);
            col.SetValue(0, 0, this._previousPoints[i].X);
            col.SetValue(1, 0, this._previousPoints[i].Y);

            col.CopyTo(colm);
            colm = homographyMat * colm;
            colm /= colm[2, 0];

            double dist = Math.Sqrt(
              Math.Pow(colm[0, 0] - imagePoints[i].X, 2) +
              Math.Pow(colm[1, 0] - imagePoints[i].Y, 2));

            if (dist < INLIER_THRESHOLD)
              inliers++;
          }

          if (inliers != imagePoints.Length)
          {
            this.ComputeAndMatch();
            this.FindObject();
          }
          else
          {
            this._previousPoints.Clear();
            this._previousPoints.Push(this._imagePoints.ToArray());
            this._imagePoints.Clear();
            this._imagePoints.Push(imagePoints);
            this._previousFrame = Memory.Frame.Clone();
          }
        }
      }
      else
      {
        this.ComputeAndMatch();
        this.FindObject();
      }
    }

    public Frame DrawObjectBorder()
    {
      Mat frame = Memory.Frame.Clone();

      if (this._inlierRatio > INLIER_USABLE_THRESHOLD)
      {
        CvInvoke.Line(frame,
          new System.Drawing.Point((int)this._imagePoints[0].X, (int)this._imagePoints[0].Y),
          new System.Drawing.Point((int)this._imagePoints[1].X, (int)this._imagePoints[1].Y),
          new Rgb(255, 0, 0).MCvScalar, 5);
        CvInvoke.Line(frame,
          new System.Drawing.Point((int)this._imagePoints[0].X, (int)this._imagePoints[0].Y),
          new System.Drawing.Point((int)this._imagePoints[2].X, (int)this._imagePoints[2].Y),
          new Rgb(0, 255, 0).MCvScalar, 5);
        CvInvoke.Line(frame,
          new System.Drawing.Point((int)this._imagePoints[2].X, (int)this._imagePoints[2].Y),
          new System.Drawing.Point((int)this._imagePoints[3].X, (int)this._imagePoints[3].Y),
          new Rgb(255, 0, 0).MCvScalar, 5);
        CvInvoke.Line(frame,
          new System.Drawing.Point((int)this._imagePoints[1].X, (int)this._imagePoints[1].Y),
          new System.Drawing.Point((int)this._imagePoints[3].X, (int)this._imagePoints[3].Y),
          new Rgb(0, 255, 0).MCvScalar, 5);
      }

      using (Image<Bgr, byte> nextFrame = frame.ToImage<Bgr, byte>())
      {
        System.Drawing.Bitmap currentFrame = nextFrame.ToBitmap();
        MemoryStream m = new MemoryStream();
        System.Drawing.Imaging.ImageFormat format =
          !this._unity ? System.Drawing.Imaging.ImageFormat.Bmp : currentFrame.RawFormat;
        currentFrame.Save(m, format);

        return new Frame(currentFrame.Height, currentFrame.Width, m.ToArray());
      }
    }
  }

  public class CameraProperties
  {
    private readonly MCvTermCriteria _termCritera = new MCvTermCriteria(30, double.Epsilon);
    private readonly Camera _cap;
    private readonly System.Drawing.Size _patternSize;
    private readonly float _squareSize;
    private bool _calibrated;
    private Mat _cameraMat, _distCoeffs;
    // private VectorOfMat _rotationVectors, _translationVectors;
    // private VectorOfVectorOfPointF _imageCoords;
    private double _err;

    public CameraProperties(Camera cap, Size patternSize, float squareSize)
    {
      this._cap = cap;
      this._patternSize = patternSize != null ? patternSize.Dims : new System.Drawing.Size(0, 0);
      this._squareSize = squareSize;
      this._calibrated = false;
      this._cameraMat = Mat.Eye(3, 3, DepthType.Cv32F, 1);
      this._distCoeffs = Mat.Zeros(5, 1, DepthType.Cv64F, 1);
      // this._rotationVectors = new VectorOfMat();
      // this._translationVectors = new VectorOfMat();
      // this._imageCoords = new VectorOfVectorOfPointF();
    }

    public CameraProperties() : this(null, null, -1) { }

    // TODO convert to non-opencv data types
    public Mat CameraMatrix { get => this._cameraMat; }
    public Mat DistortionCoefficients { get => this._distCoeffs; }
    // public VectorOfMat RotationVectors { get => this._rotationVectors; }
    // public VectorOfMat TranslationVectors { get => this._translationVectors; }
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

      /*
      this._err = CvInvoke.CalibrateCamera(objectCoords.ToArrayOfArray(),
        this._imageCoords.ToArrayOfArray(), Memory.Frame.Size, this._cameraMat,
        this._distCoeffs, CalibType.Default, _termCritera, out rvec, out tvec);

      this._rotationVectors.Push(rvec);
      this._translationVectors.Push(tvec);

      this.SaveToFile(err, this._cameraMat, this._distCoeffs, this._rotationVectors,
        this._translationVectors, objectCoord, this._imageCoords);
      */

      this._err = CvInvoke.CalibrateCamera(objectCoords.ToArrayOfArray(),
        imageCoords.ToArrayOfArray(), Memory.Frame.Size, this._cameraMat,
        this._distCoeffs, CalibType.Default, _termCritera, out Mat[] rvec, out Mat[] tvec);

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
        // fs.Write(tvec, "translationVectors");
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
      Camera capture = new Camera(0, false);
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
      Camera capture = new Camera(0, false);
      Image<Bgr, byte> img;

      String win1 = "Camera Calibration Demo";
      CvInvoke.NamedWindow(win1);

      CameraProperties cc = new CameraProperties(capture, patternSize, squareSize);
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

    public static void ReadIntrinsicsFile()
    {
      CameraProperties cc = new CameraProperties();
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
      FeaturePoints fp = FeaturePoints.ReadData(keypointsFilePath, false);
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
      FeaturePoints fp = FeaturePoints.ReadData(keypointsFilePath, false);

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
      FeaturePoints.ComputeAndSave(imageFilePath, keypointsFilePath);
      FeaturePoints fp = FeaturePoints.ReadData(keypointsFilePath, false);
      int ntracks = 0, nmatches = 0;

      Camera capture = new Camera(0, size, false);
      Frame frame;
      Image<Bgr, byte> img;

      String win1 = "Feature Tracking Demo";
      CvInvoke.NamedWindow(win1, NamedWindowType.Normal);

      for (int i = 0; ; i++)
      {
        capture.GetNextFrame();

        if (i == 0)
        {
          fp.ComputeAndMatch();
          fp.FindObject();
        }
        else
          fp.TrackObject();

        frame = fp.DrawObjectBorder();
        img = new Image<Bgr, byte>(frame.Width, frame.Height)
        {
          Bytes = frame.Image
        };

        if (fp.State == FeaturePoints.FeatureState.MATCHING)
        {
          nmatches++;
          System.Diagnostics.Debug.Write("Detecting, Describing, and Matching");
          System.Diagnostics.Debug.Write("\t\tMatches " + fp.Matches);
          System.Diagnostics.Debug.Write("\t\tInliers " + fp.Inliers);
          System.Diagnostics.Debug.WriteLine("\t\tInlier Ratio " + fp.InlierRatio);
        }
        else
        {
          ntracks++;
          System.Diagnostics.Debug.Write("Tracking");
          System.Diagnostics.Debug.WriteLine("\t\tAverage Error " + fp.TrackerAverageError);
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
