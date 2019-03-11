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
      //ARKit.Test.CaptureAndShow();

      // Test 3
      ARKit.Test.RunChessboardDemo(new ARKit.Size(4, 7));
    }
  }
}
