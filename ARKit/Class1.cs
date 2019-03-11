﻿using Emgu.CV;
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

    public Camera(int cameraId = 0, bool unity = true)
    {
      this.cap = new VideoCapture(cameraId);
      this.unity = unity;
    }

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
    private VectorOfKeyPoint _keypoints;
    private Mat _descriptors;

    public FeaturePoints() { }
    public FeaturePoints(VectorOfKeyPoint keypoints, Mat descriptors)
    {
      this._keypoints = keypoints;
      this._descriptors = descriptors;
    }

    public VectorOfKeyPoint KeyPoints { get => this._keypoints; }
    public Mat Descriptors { get => this._descriptors; }

    public static void ComputeAndSave(string imageFilePath, string keypointsFilePath)
    {
      VectorOfKeyPoint itemKeypoints = new VectorOfKeyPoint();
      Mat itemDescriptors = new Mat(), image = CvInvoke.Imread(imageFilePath);

      using (AKAZE akaze = new AKAZE())
      //using (BFMatcher matcher = new BFMatcher(DistanceType.Hamming))
      {
        akaze.DetectAndCompute(image, null, itemKeypoints, itemDescriptors, false);
      }

      using (FileStorage fs = new FileStorage(keypointsFilePath,
        FileStorage.Mode.Write | FileStorage.Mode.FormatYaml))
      {
        fs.Write(itemKeypoints.Size, "number-of-keypoints");
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

    public static FeaturePoints ReadData(string filepath)
    {
      Mat descriptors = new Mat();
      List<MKeyPoint> keypoints = new List<MKeyPoint>();

      using (FileStorage fs = new FileStorage(
        filepath, FileStorage.Mode.Read | FileStorage.Mode.FormatYaml))
      {
        int numKeypoints = fs.GetNode("number-of-keypoints").ReadInt();
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
        new VectorOfKeyPoint(keypoints.ToArray()), descriptors);
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
      Camera capture = new Camera(0, false);
      Frame frame;
      Image<Bgr, byte> img;

      String win1 = "Camera Demo";
      CvInvoke.NamedWindow(win1);

      for (; ; )
      {
        frame = capture.GetNextFrame();
        img = new Image<Bgr, byte>(frame.Width, frame.Height)
        {
          Bytes = frame.Image
        };

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
      FeaturePoints fp = FeaturePoints.ReadData(keypointsFilePath);
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
  }
}
