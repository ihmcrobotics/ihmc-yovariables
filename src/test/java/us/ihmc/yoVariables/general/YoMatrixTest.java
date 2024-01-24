package us.ihmc.yoVariables.general;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.MatrixFeatures_DDRM;
import org.ejml.dense.row.RandomMatrices_DDRM;
import org.junit.jupiter.api.Test;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

public class YoMatrixTest
{
   private static final double EPSILON = 1.0e-10;

   @Test
   public void testSimpleYoMatrixExample()
   {
      int maxNumberOfRows = 4;
      int maxNumberOfColumns = 8;
      YoRegistry registry = new YoRegistry("testRegistry");
      YoMatrix yoMatrix = new YoMatrix("testMatrix", maxNumberOfRows, maxNumberOfColumns, registry);
      assertEquals(maxNumberOfRows, yoMatrix.getNumberOfRows());
      assertEquals(maxNumberOfColumns, yoMatrix.getNumberOfColumns());

      DMatrixRMaj denseMatrix = new DMatrixRMaj(maxNumberOfRows, maxNumberOfColumns);
      yoMatrix.get(denseMatrix);

      assertTrue(MatrixFeatures_DDRM.isZeros(denseMatrix, EPSILON));

      Random random = new Random(1984L);

      DMatrixRMaj randomMatrix = RandomMatrices_DDRM.rectangle(maxNumberOfRows, maxNumberOfColumns, random);
      yoMatrix.set(randomMatrix);

      DMatrixRMaj checkMatrix = new DMatrixRMaj(maxNumberOfRows, maxNumberOfColumns);
      yoMatrix.get(checkMatrix);

      assertArrayEquals(randomMatrix.getData(), checkMatrix.getData(), EPSILON);

      assertEquals(registry.findVariable("testMatrix_0_0").getValueAsDouble(), checkMatrix.get(0, 0), EPSILON);
   }

   @Test
   public void testYoMatrixDimensioning()
   {
      int maxNumberOfRows = 4;
      int maxNumberOfColumns = 8;
      String name = "testMatrix";

      YoRegistry registry = new YoRegistry("testRegistry");
      YoMatrix yoMatrix = new YoMatrix(name, maxNumberOfRows, maxNumberOfColumns, registry);

      int smallerRows = maxNumberOfRows - 2;
      int smallerColumns = maxNumberOfColumns - 3;
      DMatrixRMaj denseMatrix = new DMatrixRMaj(smallerRows, smallerColumns);

      try
      {
         yoMatrix.get(denseMatrix);
         fail("Should throw an exception if the size isn't right!");
      }
      catch (Exception e)
      {
         // good
      }

      yoMatrix.getAndReshape(denseMatrix);
      assertTrue(MatrixFeatures_DDRM.isZeros(denseMatrix, EPSILON));
      assertEquals(maxNumberOfRows, denseMatrix.getNumRows());
      assertEquals(maxNumberOfColumns, denseMatrix.getNumCols());

      assertEquals(maxNumberOfRows, yoMatrix.getNumberOfRows());
      assertEquals(maxNumberOfColumns, yoMatrix.getNumberOfColumns());

      Random random = new Random(1984L);

      DMatrixRMaj randomMatrix = RandomMatrices_DDRM.rectangle(maxNumberOfRows, maxNumberOfColumns, random);
      yoMatrix.set(randomMatrix);

      DMatrixRMaj checkMatrix = new DMatrixRMaj(maxNumberOfRows, maxNumberOfColumns);
      yoMatrix.get(checkMatrix);

      assertArrayEquals(randomMatrix.getData(), checkMatrix.getData(), EPSILON);

      DMatrixRMaj smallerMatrix = RandomMatrices_DDRM.rectangle(smallerRows, smallerColumns, random);
      yoMatrix.set(smallerMatrix);

      assertEquals(smallerRows, smallerMatrix.getNumRows());
      assertEquals(smallerColumns, smallerMatrix.getNumCols());

      assertEquals(smallerRows, yoMatrix.getNumberOfRows());
      assertEquals(smallerColumns, yoMatrix.getNumberOfColumns());

      DMatrixRMaj checkMatrix2 = new DMatrixRMaj(1, 1);
      yoMatrix.getAndReshape(checkMatrix2);

      assertArrayEquals(smallerMatrix.getData(), checkMatrix2.getData(), EPSILON);

      checkMatrixYoVariablesEqualsCheckMatrixAndOutsideValuesAreNaN(name, maxNumberOfRows, maxNumberOfColumns, checkMatrix2, registry);
   }

   @Test
   public void testYoMatrixSetToZero()
   {
      int maxNumberOfRows = 4;
      int maxNumberOfColumns = 8;
      String name = "testMatrixForZero";

      YoRegistry registry = new YoRegistry("testRegistry");
      YoMatrix yoMatrix = new YoMatrix(name, maxNumberOfRows, maxNumberOfColumns, registry);

      Random random = new Random(1984L);

      DMatrixRMaj randomMatrix = RandomMatrices_DDRM.rectangle(maxNumberOfRows, maxNumberOfColumns, random);
      yoMatrix.set(randomMatrix);

      int numberOfRows = 2;
      int numberOfColumns = 6;
      yoMatrix.setToZero(numberOfRows, numberOfColumns);

      DMatrixRMaj zeroMatrix = new DMatrixRMaj(numberOfRows, numberOfColumns);
      checkMatrixYoVariablesEqualsCheckMatrixAndOutsideValuesAreNaN(name, maxNumberOfRows, maxNumberOfColumns, zeroMatrix, registry);
   }

   @Test
   public void testYoMatrixSetTooBig()
   {
      int maxNumberOfRows = 4;
      int maxNumberOfColumns = 8;
      String name = "testMatrix";
      YoRegistry registry = new YoRegistry("testRegistry");
      YoMatrix yoMatrix = new YoMatrix(name, maxNumberOfRows, maxNumberOfColumns, registry);

      DMatrixRMaj tooBigMatrix = new DMatrixRMaj(maxNumberOfRows + 1, maxNumberOfColumns);

      try
      {
         yoMatrix.set(tooBigMatrix);
         fail("Too Big");
      }
      catch (RuntimeException e)
      {
         // good
      }

      tooBigMatrix = new DMatrixRMaj(maxNumberOfRows, maxNumberOfColumns + 1);

      try
      {
         yoMatrix.set(tooBigMatrix);
         fail("Too Big");
      }
      catch (RuntimeException e)
      {
         // good
      }

      // Test a 0 X Big Matrix
      DMatrixRMaj okMatrix = new DMatrixRMaj(0, maxNumberOfColumns + 10);
      yoMatrix.set(okMatrix);
      assertMatrixYoVariablesAreNaN(name, maxNumberOfRows, maxNumberOfColumns, registry);

      DMatrixRMaj checkMatrix = new DMatrixRMaj(1, 1);
      yoMatrix.getAndReshape(checkMatrix);

      assertEquals(0, checkMatrix.getNumRows());
      assertEquals(maxNumberOfColumns + 10, checkMatrix.getNumCols());

      // Test a Big X 0 Matrix

      okMatrix = new DMatrixRMaj(maxNumberOfRows + 10, 0);
      yoMatrix.set(okMatrix);
      assertMatrixYoVariablesAreNaN(name, maxNumberOfRows, maxNumberOfColumns, registry);

      checkMatrix = new DMatrixRMaj(1, 1);
      yoMatrix.getAndReshape(checkMatrix);

      assertEquals(maxNumberOfRows + 10, checkMatrix.getNumRows());
      assertEquals(0, checkMatrix.getNumCols());
   }

   private void checkMatrixYoVariablesEqualsCheckMatrixAndOutsideValuesAreNaN(String name,
                                                                              int maxNumberOfRows,
                                                                              int maxNumberOfColumns,
                                                                              DMatrixRMaj checkMatrix,
                                                                              YoRegistry registry)
   {
      int smallerRows = checkMatrix.getNumRows();
      int smallerColumns = checkMatrix.getNumCols();

      // Make sure the values are correct, including values outside the range should be NaN:
      for (int row = 0; row < maxNumberOfRows; row++)
      {
         for (int column = 0; column < maxNumberOfColumns; column++)
         {
            YoDouble variable = (YoDouble) registry.findVariable(name + "_" + row + "_" + column);

            if ((row < smallerRows) && (column < smallerColumns))
            {
               assertEquals(checkMatrix.get(row, column), variable.getDoubleValue(), EPSILON);
            }
            else
            {
               assertTrue(Double.isNaN(variable.getDoubleValue()));
            }
         }
      }
   }

   private void assertMatrixYoVariablesAreNaN(String name, int maxNumberOfRows, int maxNumberOfColumns, YoRegistry registry)
   {
      for (int row = 0; row < maxNumberOfRows; row++)
      {
         for (int column = 0; column < maxNumberOfColumns; column++)
         {
            YoDouble variable = (YoDouble) registry.findVariable(name + "_" + row + "_" + column);
            assertTrue(Double.isNaN(variable.getDoubleValue()));
         }
      }
   }
}