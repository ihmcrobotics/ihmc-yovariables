package us.ihmc.yoVariables.math;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.RandomMatrices_DDRM;
import org.junit.jupiter.api.Test;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

public class YoMatrixTest
{
   private static final double EPSILON = 1.0e-10;
   private static final int ITERATIONS = 1000;

   @Test
   public void testSimpleYoMatrixRefactorExample()
   {
      Random random = new Random(1984L);
      int rowSize = random.nextInt(5, 10);
      int columnSize = random.nextInt(5, 10);

      YoRegistry registry = new YoRegistry("testRegistry");
      YoMatrix matrix = new YoMatrix("testMatrix", rowSize, columnSize, registry);
      assertEquals(rowSize, matrix.getNumRows());
      assertEquals(columnSize, matrix.getNumCols());

      DMatrixRMaj expected = RandomMatrices_DDRM.rectangle(rowSize, columnSize, random);
      matrix.set(expected);
      DMatrixRMaj actual = new DMatrixRMaj(rowSize, columnSize);
      matrix.get(actual);
      assertArrayEquals(expected.getData(), actual.getData(), EPSILON);
      assertEquals(rowSize * columnSize, matrix.getNumElements());

      // Make smaller
      int smallerRowSize = random.nextInt(1, rowSize);
      int smallerColumnSize = random.nextInt(1, columnSize);

      DMatrixRMaj smallerExpected = RandomMatrices_DDRM.rectangle(smallerRowSize, smallerColumnSize, random);
      matrix.set(smallerExpected);
      DMatrixRMaj smallerActual = new DMatrixRMaj(smallerRowSize, smallerColumnSize);
      matrix.get(smallerActual);
      assertArrayEquals(smallerExpected.getData(), smallerActual.getData(), EPSILON);
      assertEquals(smallerRowSize * smallerColumnSize, matrix.getNumElements());
   }

   @Test
   public void testConstructorsWithNamesAndDescriptions()
   {
      // Passing in column names without row names -- should throw exception
      try
      {
         new YoMatrix("testMatrix", 4, 4, null, new String[] {"col1", "col2", "col3", "col4"}, new YoRegistry("testRegistry"));
         new YoMatrix("testMatrix", "testDescription", 4, 4, null, new String[] {"col1", "col2", "col3", "col4"}, new YoRegistry("testRegistry"));
         fail("Should have thrown an exception");
      }
      catch (RuntimeException e)
      {
         // good
      }

      // Passing in just row names, but with more than one column -- should throw exception
      try
      {
         new YoMatrix("testMatrix", 4, 4, new String[] {"row1", "row2", "row3", "row4"}, new YoRegistry("testRegistry"));
         new YoMatrix("testMatrix", "testDescription", 4, 4, new String[] {"row1", "row2", "row3", "row4"}, null, new YoRegistry("testRegistry"));
         fail("Should have thrown an exception");
      }
      catch (RuntimeException e)
      {
         // good
      }

      // Passing in row/column name arrays of different length to the number of rows/columns -- should throw exception
      // Row names are fine, column names are too short
      try
      {
         new YoMatrix("testMatrix", 4, 4, new String[] {"row1", "row2", "row3", "row4"}, new String[] {"col1", "col2"}, new YoRegistry("testRegistry"));
         new YoMatrix("testMatrix",
                      "testDescription",
                      4,
                      4,
                      new String[] {"row1", "row2", "row3", "row4"},
                      new String[] {"col1", "col2"},
                      new YoRegistry("testRegistry"));
         fail("Should have thrown an exception");
      }
      catch (RuntimeException e)
      {
         // good
      }
      // Row names are too short, column names are fine
      try
      {
         new YoMatrix("testMatrix", 4, 4, new String[] {"row1", "row2"}, new String[] {"col1", "col2", "col3", "col4"}, new YoRegistry("testRegistry"));
         new YoMatrix("testMatrix",
                      "testDescription",
                      4,
                      4,
                      new String[] {"row1", "row2"},
                      new String[] {"col1", "col2", "col3", "col4"},
                      new YoRegistry("testRegistry"));
         fail("Should have thrown an exception");
      }
      catch (RuntimeException e)
      {
         // good
      }
   }

   @Test
   public void testGetDimensioning()
   {
      Random random = new Random(1984L);

      for (int i = 0; i < ITERATIONS; ++i)
      {
         int rowSize = random.nextInt(5, 10);
         int columnSize = random.nextInt(5, 10);

         String name = "testMatrix";
         YoRegistry registry = new YoRegistry("testRegistry");
         YoMatrix matrix = new YoMatrix(name, rowSize, columnSize, registry);

         int smallerRowSize = random.nextInt(1, rowSize);
         int smallerColumnSize = random.nextInt(1, columnSize);
         DMatrixRMaj smallerMatrix = new DMatrixRMaj(smallerRowSize, smallerColumnSize);
         // Attempt to pack with mismatched matrix sizes
         try
         {
            matrix.get(smallerMatrix);
            fail("Should have thrown an exception");
         }
         catch (Exception e)
         {
            // good
         }

         matrix.getAndReshape(smallerMatrix);
         assertEquals(rowSize, smallerMatrix.getNumRows());
         assertEquals(columnSize, smallerMatrix.getNumCols());
         assertEquals(matrix.getNumRows(), smallerMatrix.getNumRows());
         assertEquals(matrix.getNumCols(), smallerMatrix.getNumCols());

         // Generate new random matrix with larger row and columns sizes than small, but within maximums
         int largerRowSize = random.nextInt(smallerRowSize, rowSize);
         int largerColumnSize = random.nextInt(smallerColumnSize, columnSize);
         DMatrixRMaj expected = RandomMatrices_DDRM.rectangle(largerRowSize, largerColumnSize, random);
         matrix.set(expected);

         DMatrixRMaj actual = new DMatrixRMaj(largerRowSize, largerColumnSize);
         matrix.get(actual);
         assertArrayEquals(expected.getData(), actual.getData(), EPSILON);
      }
   }

   @Test
   public void testSetDimensioning()
   {
      Random random = new Random(1984L);

      for (int i = 0; i < ITERATIONS; ++i)
      {
         int rowSize = random.nextInt(5, 10);
         int columnSize = random.nextInt(5, 10);

         String name = "testMatrix";
         YoRegistry registry = new YoRegistry("testRegistry");
         YoMatrix matrix = new YoMatrix(name, rowSize, columnSize, registry);

         // + 1 because lower bounds are inclusive
         int largerRowSize = random.nextInt(rowSize + 1, 2 * rowSize);
         int largerColumnSize = random.nextInt(columnSize + 1, 2 * columnSize);
         DMatrixRMaj largerMatrix = new DMatrixRMaj(largerRowSize, largerColumnSize);
         try
         {
            matrix.set(largerMatrix);
            fail("Should have thrown an exception");
         }
         catch (RuntimeException e)
         {
            // good
         }
      }
   }

   @Test
   public void testZero()
   {
      Random random = new Random(1984L);

      for (int i = 0; i < ITERATIONS; ++i)
      {
         int rowSize = random.nextInt(5, 10);
         int columnSize = random.nextInt(5, 10);

         String name = "testMatrixForZero";
         YoRegistry registry = new YoRegistry("testRegistry");
         YoMatrix matrix = new YoMatrix(name, rowSize, columnSize, registry);

         DMatrixRMaj randomMatrix = RandomMatrices_DDRM.rectangle(rowSize, columnSize, random);
         matrix.set(randomMatrix);

         int smallerRowSize = random.nextInt(1, rowSize);
         int smallerColumnSize = random.nextInt(1, columnSize);
         matrix.zero(smallerRowSize, smallerColumnSize);

         DMatrixRMaj zeroMatrix = new DMatrixRMaj(smallerRowSize, smallerColumnSize);
         checkMatrixYoVariablesEqualsCheckMatrixAndOutsideValuesAreNaN(name, rowSize, columnSize, zeroMatrix, registry);
      }
   }

   @Test
   public void testNaN()
   {
      Random random = new Random(1984L);

      for (int i = 0; i < ITERATIONS; ++i)
      {
         int rowSize = random.nextInt(5, 10);
         int columnSize = random.nextInt(5, 10);
         YoMatrix matrix = new YoMatrix("testMatrix", rowSize, columnSize, new YoRegistry("testRegistry"));

         DMatrixRMaj randomMatrix = RandomMatrices_DDRM.rectangle(rowSize, columnSize, random);
         matrix.set(randomMatrix);
         assertFalse(matrix.containsNaN());

         matrix.setToNaN(rowSize, columnSize);
         assertTrue(matrix.containsNaN());
      }
   }

   @Test
   public void testScale()
   {
      Random random = new Random(1984L);

      // In-place scale
      for (int i = 0; i < ITERATIONS; ++i)
      {
         int rowSize = random.nextInt(5, 10);
         int columnSize = random.nextInt(5, 10);
         YoMatrix matrix = new YoMatrix("testMatrix", rowSize, columnSize, new YoRegistry("testRegistry"));

         DMatrixRMaj randomMatrix = RandomMatrices_DDRM.rectangle(rowSize, columnSize, random);
         matrix.set(randomMatrix);

         double scale = random.nextDouble();
         matrix.scale(scale);

         DMatrixRMaj expected = new DMatrixRMaj(rowSize, columnSize);
         CommonOps_DDRM.scale(scale, randomMatrix, expected);
         DMatrixRMaj actual = new DMatrixRMaj(rowSize, columnSize);
         matrix.get(actual);
         assertArrayEquals(expected.getData(), actual.getData(), EPSILON);
      }

      // Scale and set
      for (int j = 0; j < ITERATIONS; ++j)
      {
         int rowSize = random.nextInt(5, 10);
         int columnSize = random.nextInt(5, 10);
         YoMatrix matrix = new YoMatrix("testMatrix", rowSize, columnSize, new YoRegistry("testRegistry"));

         double scale = random.nextDouble();
         DMatrixRMaj randomMatrix = RandomMatrices_DDRM.rectangle(rowSize, columnSize, random);
         matrix.scale(scale, randomMatrix);

         DMatrixRMaj expected = new DMatrixRMaj(rowSize, columnSize);
         CommonOps_DDRM.scale(scale, randomMatrix, expected);

         DMatrixRMaj actual = new DMatrixRMaj(rowSize, columnSize);
         matrix.get(actual);
         assertArrayEquals(expected.getData(), actual.getData(), EPSILON);
      }
   }

   @Test
   public void testScaleFailureCases()
   {
      Random random = new Random(1984L);

      // If the matrix is not the same size as the input matrix, it should throw an exception
      for (int i = 0; i < ITERATIONS; ++i)
      {
         int minDimension = 1;
         int maxDimension = 5;
         YoMatrix matrix = new YoMatrix("testMatrix",
                                        random.nextInt(minDimension, maxDimension),
                                        random.nextInt(minDimension, maxDimension),
                                        new YoRegistry("testRegistry"));

         DMatrixRMaj matrixToScale = RandomMatrices_DDRM.rectangle(random.nextInt(minDimension, maxDimension),
                                                                   random.nextInt(minDimension, maxDimension),
                                                                   random);

         if (matrix.getNumRows() != matrixToScale.getNumRows() || matrix.getNumCols() != matrixToScale.getNumCols())
         {
            try
            {
               matrix.scale(random.nextDouble(), matrixToScale);
               fail("Should have thrown an exception");
            }
            catch (RuntimeException e)
            {
               // good
            }
         }
         else
         {
            matrix.scale(random.nextDouble(), matrixToScale);
         }
      }
   }

   @Test
   public void testAdd()
   {
      Random random = new Random(1984L);

      // A + B
      for (int i = 0; i < ITERATIONS; ++i)
      {
         int rowSize = random.nextInt(5, 10);
         int columnSize = random.nextInt(5, 10);
         YoMatrix matrix = new YoMatrix("testMatrix", rowSize, columnSize, new YoRegistry("testRegistry"));

         DMatrixRMaj A = RandomMatrices_DDRM.rectangle(rowSize, columnSize, random);
         DMatrixRMaj B = RandomMatrices_DDRM.rectangle(rowSize, columnSize, random);
         matrix.add(A, B);

         DMatrixRMaj expected = new DMatrixRMaj(rowSize, columnSize);
         CommonOps_DDRM.add(A, B, expected);
         DMatrixRMaj actual = new DMatrixRMaj(rowSize, columnSize);
         matrix.get(actual);
         assertArrayEquals(expected.getData(), actual.getData(), EPSILON);
      }

      // A + beta * B
      for (int j = 0; j < ITERATIONS; ++j)
      {
         int rowSize = random.nextInt(5, 10);
         int columnSize = random.nextInt(5, 10);
         YoMatrix matrix = new YoMatrix("testMatrix", rowSize, columnSize, new YoRegistry("testRegistry"));

         DMatrixRMaj A = RandomMatrices_DDRM.rectangle(rowSize, columnSize, random);
         DMatrixRMaj B = RandomMatrices_DDRM.rectangle(rowSize, columnSize, random);
         double beta = random.nextDouble();
         matrix.add(A, beta, B);

         DMatrixRMaj expected = new DMatrixRMaj(rowSize, columnSize);
         CommonOps_DDRM.add(A, beta, B, expected);
         DMatrixRMaj actual = new DMatrixRMaj(rowSize, columnSize);
         matrix.get(actual);
         assertArrayEquals(expected.getData(), actual.getData(), EPSILON);
      }

      // alpha * A + beta * B
      for (int k = 0; k < ITERATIONS; ++k)
      {
         int rowSize = random.nextInt(5, 10);
         int columnSize = random.nextInt(5, 10);
         YoMatrix matrix = new YoMatrix("testMatrix", rowSize, columnSize, new YoRegistry("testRegistry"));

         DMatrixRMaj A = RandomMatrices_DDRM.rectangle(rowSize, columnSize, random);
         DMatrixRMaj B = RandomMatrices_DDRM.rectangle(rowSize, columnSize, random);
         double alpha = random.nextDouble();
         double beta = random.nextDouble();
         matrix.add(alpha, A, beta, B);

         DMatrixRMaj expected = new DMatrixRMaj(rowSize, columnSize);
         CommonOps_DDRM.add(alpha, A, beta, B, expected);
         DMatrixRMaj actual = new DMatrixRMaj(rowSize, columnSize);
         matrix.get(actual);
         assertArrayEquals(expected.getData(), actual.getData(), EPSILON);
      }
   }

   @Test
   public void testAddFailureCases()
   {
      Random random = new Random(1984L);

      for (int i = 0; i < ITERATIONS; ++i)
      {
         int minDimension = 1;
         int maxDimension = 5;
         YoMatrix matrix = new YoMatrix("testMatrix",
                                        random.nextInt(minDimension, maxDimension),
                                        random.nextInt(minDimension, maxDimension),
                                        new YoRegistry("testRegistry"));

         DMatrixRMaj A = RandomMatrices_DDRM.rectangle(random.nextInt(minDimension, maxDimension), random.nextInt(minDimension, maxDimension), random);
         DMatrixRMaj B = RandomMatrices_DDRM.rectangle(random.nextInt(minDimension, maxDimension), random.nextInt(minDimension, maxDimension), random);

         // Dimensions of A and B do not match
         if (A.getNumRows() != B.getNumRows() || A.getNumCols() != B.getNumCols())
         {
            try
            {
               matrix.add(A, B);
               fail("Should have thrown an exception");
            }
            catch (RuntimeException e)
            {
               // good
            }
         }
         // Dimensions of one or both of A and B do not match dimensions of matrix
         else if (matrix.getNumRows() != A.getNumRows() || matrix.getNumCols() != A.getNumCols() || matrix.getNumRows() != B.getNumRows()
                  || matrix.getNumCols() != B.getNumCols())
         {
            try
            {
               matrix.add(A, B);
               fail("Should have thrown an exception");
            }
            catch (RuntimeException e)
            {
               // good
            }
         }
         else
         {
            matrix.add(A, B);
         }
      }
   }

   @Test
   public void testAddEquals()
   {
      Random random = new Random(1984L);

      // add A
      for (int i = 0; i < ITERATIONS; ++i)
      {
         int rowSize = random.nextInt(5, 10);
         int columnSize = random.nextInt(5, 10);
         YoMatrix matrix = new YoMatrix("testMatrix", rowSize, columnSize, new YoRegistry("testRegistry"));
         DMatrixRMaj randomMatrix = RandomMatrices_DDRM.rectangle(rowSize, columnSize, random);
         matrix.set(randomMatrix);

         DMatrixRMaj A = RandomMatrices_DDRM.rectangle(rowSize, columnSize, random);
         matrix.addEquals(A);

         DMatrixRMaj expected = new DMatrixRMaj(rowSize, columnSize);
         CommonOps_DDRM.add(randomMatrix, A, expected);
         DMatrixRMaj actual = new DMatrixRMaj(rowSize, columnSize);
         matrix.get(actual);
         assertArrayEquals(expected.getData(), actual.getData(), EPSILON);
      }

      // add alpha * A
      for (int j = 0; j < ITERATIONS; ++j)
      {
         int rowSize = random.nextInt(5, 10);
         int columnSize = random.nextInt(5, 10);
         YoMatrix matrix = new YoMatrix("testMatrix", rowSize, columnSize, new YoRegistry("testRegistry"));
         DMatrixRMaj randomMatrix = RandomMatrices_DDRM.rectangle(rowSize, columnSize, random);
         matrix.set(randomMatrix);

         DMatrixRMaj A = RandomMatrices_DDRM.rectangle(rowSize, columnSize, random);
         double alpha = random.nextDouble();
         matrix.addEquals(alpha, A);

         DMatrixRMaj expected = new DMatrixRMaj(rowSize, columnSize);
         CommonOps_DDRM.add(randomMatrix, alpha, A, expected);
         DMatrixRMaj actual = new DMatrixRMaj(rowSize, columnSize);
         matrix.get(actual);
         assertArrayEquals(expected.getData(), actual.getData(), EPSILON);
      }
   }

   @Test
   public void testAddEqualsFailureCases()
   {
      Random random = new Random(1984L);

      // Dimensions of A do not match dimensions of matrix
      for (int i = 0; i < ITERATIONS; ++i)
      {
         int minDimension = 1;
         int maxDimension = 5;
         YoMatrix matrix = new YoMatrix("testMatrix",
                                        random.nextInt(minDimension, maxDimension),
                                        random.nextInt(minDimension, maxDimension),
                                        new YoRegistry("testRegistry"));

         DMatrixRMaj A = RandomMatrices_DDRM.rectangle(random.nextInt(minDimension, maxDimension), random.nextInt(minDimension, maxDimension), random);

         if (matrix.getNumRows() != A.getNumRows() || matrix.getNumCols() != A.getNumCols())
         {
            try
            {
               matrix.addEquals(A);
               fail("Should have thrown an exception");
            }
            catch (RuntimeException e)
            {
               // good
            }
         }
         else
         {
            matrix.addEquals(A);
         }
      }
   }

   @Test
   public void testSafeSetAndGetWithIndices()
   {
      Random random = new Random(1984L);

      for (int i = 0; i < ITERATIONS; ++i)
      {
         int rowSize = random.nextInt(5, 10);
         int columnSize = random.nextInt(5, 10);
         YoMatrix matrix = new YoMatrix("testMatrix", rowSize, columnSize, new YoRegistry("testRegistry"));

         DMatrixRMaj randomMatrix = RandomMatrices_DDRM.rectangle(rowSize, columnSize, random);
         matrix.set(randomMatrix);

         for (int row = 0; row < rowSize; row++)
         {
            for (int column = 0; column < columnSize; column++)
            {
               double value = random.nextDouble();
               matrix.set(row, column, value);
               assertEquals(value, matrix.get(row, column), EPSILON);
            }
         }
      }
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
}