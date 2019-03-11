using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ARKit
{
  public class Size
  {
    private System.Drawing.Size size;

    public Size(int height, int width)
    {
      this.size = new System.Drawing.Size(height, width);
    }

    public System.Drawing.Size Dims => this.size;
  }

  public class Frame
  {
    private readonly int height, width;
    private readonly byte[] image;

    public Frame(int height, int width, byte[] image)
    {
      this.height = height;
      this.width = width;
      this.image = image;
    }

    public int Height => this.height;
    public int Width => this.width;
    public byte[] Image => this.image;
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
      using (Image<Bgr, byte> nextFrame = this.cap.QueryFrame().ToImage<Bgr, byte>())
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

  public class ChessboardDemo
  {
    private readonly bool unity;
    private readonly System.Drawing.Size patternSize;
    private readonly Image<Bgr, byte> image;
    private Emgu.CV.Util.VectorOfPointF centers = new Emgu.CV.Util.VectorOfPointF();

    public ChessboardDemo(Frame frame, bool unity = true)
    {
      // converting byte[] to Image, referenced from https://stackoverflow.com/questions/29153967/convert-a-byte-into-an-emgu-opencv-image
      this.image = new Image<Bgr, byte>(frame.Width, frame.Height)
      {
        Bytes = frame.Image
      };
      this.unity = unity;
    }

    public ChessboardDemo(Frame frame, Size patternSize, bool unity = true) : this(frame, unity)
    {
      this.patternSize = patternSize.Dims;
    }

    public Frame RunDemo()
    {
      Mat f = this.image.Mat;
      bool patternFound = CvInvoke.FindChessboardCorners(f, this.patternSize, centers);

      CvInvoke.DrawChessboardCorners(f, this.patternSize, centers, patternFound);

      System.Drawing.Bitmap currentFrame = f.ToImage<Bgr, byte>().ToBitmap();
      MemoryStream m = new MemoryStream();
      System.Drawing.Imaging.ImageFormat format =
          !this.unity ? System.Drawing.Imaging.ImageFormat.Bmp : currentFrame.RawFormat;
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
        frame = (new ChessboardDemo(capture.GetNextFrame(), patternSize, false)).RunDemo();
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
  }
}
