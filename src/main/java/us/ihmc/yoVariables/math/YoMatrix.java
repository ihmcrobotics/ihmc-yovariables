package us.ihmc.yoVariables.math;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixD1;
import org.ejml.data.Matrix;
import org.ejml.data.MatrixType;
import org.ejml.data.ReshapeMatrix;
import org.ejml.ops.MatrixIO;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * Holds a matrix of YoVariables so that an entire matrix can be rewound.
 * <p>
 * Has a maximum number of rows and columns and an actual number of rows and columns. If set with a smaller matrix, then the actual size will be the size of the
 * passed in matrix -- extra entries will be set to NaN. Optionally, one can provide row and column names, which will be used to name the constituent YoDoubles.
 * </p>
 *
 * @author Jerry Pratt
 * @author James Foster
 */
public class YoMatrix implements DMatrix, ReshapeMatrix
{
   private static final long serialVersionUID = 2156411740647948028L;

   private final int maxNumberOfRows, maxNumberOfColumns;

   private final YoInteger numberOfRows, numberOfColumns;
   private final YoDouble[][] variables;

   /**
    * Create a YoMatrix with the given name, number of rows, and number of columns. The constituent YoDoubles are named by index.
    *
    * @param name               common name component of the YoMatrix entries.
    * @param maxNumberOfRows    maximum number of rows in the YoMatrix.
    * @param maxNumberOfColumns maximum number of columns in the YoMatrix.
    * @param registry           YoRegistry to register the YoMatrix with.
    */
   public YoMatrix(String name, int maxNumberOfRows, int maxNumberOfColumns, YoRegistry registry)
   {
      this(name, null, maxNumberOfRows, maxNumberOfColumns, null, null, registry);
   }

   /**
    * Create a YoMatrix with the given name, number of rows, and number of columns. The constituent YoDoubles are named by row name.
    * <p>
    * The number of columns must be equal to 1, that is, the YoMatrix must be a column vector. Otherwise, it is difficult to provide an API that will name the
    * YoDoubles uniquely.
    * </p>
    *
    * @param name               common name component of the YoMatrix entries.
    * @param maxNumberOfRows    maximum number of rows in the YoMatrix.
    * @param maxNumberOfColumns maximum number of columns in the YoMatrix.
    * @param rowNames           names of the rows.
    * @param registry           YoRegistry to register the YoMatrix with.
    */
   public YoMatrix(String name, int maxNumberOfRows, int maxNumberOfColumns, String[] rowNames, YoRegistry registry)
   {
      this(name, null, maxNumberOfRows, maxNumberOfColumns, rowNames, null, registry);
   }

   /**
    * Create a YoMatrix with the given name, number of rows, and number of columns. The constituent YoDoubles are named by the entries in {@code rowNames} and
    * {@code columnNames}.
    * <p>
    * NOTE: the entries in {@code rowNames} and {@code columnNames} must be unique. Otherwise, the YoDoubles will not have unique names.
    * </p>
    *
    * @param name               common name component of the YoMatrix entries.
    * @param maxNumberOfRows    maximum number of rows in the YoMatrix.
    * @param maxNumberOfColumns maximum number of columns in the YoMatrix.
    * @param rowNames           names of the rows.
    * @param columnNames        names of the columns.
    * @param registry           YoRegistry to register the YoMatrix with.
    */
   public YoMatrix(String name, int maxNumberOfRows, int maxNumberOfColumns, String[] rowNames, String[] columnNames, YoRegistry registry)
   {
      this(name, null, maxNumberOfRows, maxNumberOfColumns, rowNames, columnNames, registry);
   }

   /**
    * Create a YoMatrix with the given name, number of rows, and number of columns. The constituent YoDoubles are named by index.
    *
    * @param name               common name component of the YoMatrix entries.
    * @param description        description of this matrix's purpose.
    * @param maxNumberOfRows    maximum number of rows in the YoMatrix.
    * @param maxNumberOfColumns maximum number of columns in the YoMatrix.
    * @param registry           YoRegistry to register the YoMatrix with.
    */
   public YoMatrix(String name, String description, int maxNumberOfRows, int maxNumberOfColumns, YoRegistry registry)
   {
      this(name, description, maxNumberOfRows, maxNumberOfColumns, null, null, registry);
   }

   /**
    * Create a YoMatrix with the given name, number of rows, and number of columns. The constituent YoDoubles are named by row name.
    * <p>
    * The number of columns must be equal to 1, that is, the YoMatrix must be a column vector. Otherwise, it is difficult to provide an API that will name the
    * YoDoubles uniquely.
    * </p>
    *
    * @param name               common name component of the YoMatrix entries.
    * @param description        description of this matrix's purpose.
    * @param maxNumberOfRows    maximum number of rows in the YoMatrix.
    * @param maxNumberOfColumns maximum number of columns in the YoMatrix.
    * @param rowNames           names of the rows.
    * @param registry           YoRegistry to register the YoMatrix with.
    */
   public YoMatrix(String name, String description, int maxNumberOfRows, int maxNumberOfColumns, String[] rowNames, YoRegistry registry)
   {
      this(name, description, maxNumberOfRows, maxNumberOfColumns, rowNames, null, registry);
   }

   /**
    * Create a YoMatrix with the given name, number of rows, and number of columns. The constituent YoDoubles are named by the entries in {@code rowNames} and
    * {@code columnNames}.
    * <p>
    * NOTE: the entries in {@code rowNames} and {@code columnNames} must be unique. Otherwise, the YoDoubles will not have unique names.
    * </p>
    *
    * @param name               common name component of the YoMatrix entries.
    * @param description        description of this matrix's purpose.
    * @param maxNumberOfRows    maximum number of rows in the YoMatrix.
    * @param maxNumberOfColumns maximum number of columns in the YoMatrix.
    * @param rowNames           names of the rows.
    * @param columnNames        names of the columns.
    * @param registry           YoRegistry to register the YoMatrix with.
    */
   public YoMatrix(String name, String description, int maxNumberOfRows, int maxNumberOfColumns, String[] rowNames, String[] columnNames, YoRegistry registry)
   {
      this.maxNumberOfRows = maxNumberOfRows;
      this.maxNumberOfColumns = maxNumberOfColumns;

      this.numberOfRows = new YoInteger(name + "NumRows", registry);
      this.numberOfColumns = new YoInteger(name + "NumCols", registry);

      this.numberOfRows.set(maxNumberOfRows);
      this.numberOfColumns.set(maxNumberOfColumns);

      variables = new YoDouble[maxNumberOfRows][maxNumberOfColumns];

      for (int row = 0; row < maxNumberOfRows; row++)
      {
         for (int column = 0; column < maxNumberOfColumns; column++)
         {
            switch (checkNames(rowNames, columnNames))
            {
               case NONE:
               {
                  variables[row][column] = new YoDouble(getFieldName(name, row, column), description, registry);
                  variables[row][column].setToNaN();
                  break;
               }
               case ROWS:
               {
                  if (maxNumberOfColumns > 1)
                     throw new IllegalArgumentException(
                           "The YoMatrix must be a column vector if only row names are provided, else unique names cannot be generated.");

                  variables[row][column] = new YoDouble(getFieldName(name, rowNames[row], ""), description, registry);
                  variables[row][column].setToNaN();
                  break;
               }
               case ROWS_AND_COLUMNS:
               {
                  variables[row][column] = new YoDouble(getFieldName(name, rowNames[row], columnNames[column]), description, registry);
                  variables[row][column].setToNaN();
                  break;
               }
            }
         }
      }
   }

   public static String getFieldName(String prefix, int row, int column)
   {
      return getFieldName(prefix, "_" + row, "_" + column);
   }

   public static String getFieldName(String prefix, String rowName, String columName)
   {
      return prefix + rowName + columName;
   }

   /**
    * Enum used to determine what names have been provided to the YoMatrix.
    */
   private enum NamesProvided
   {
      NONE, ROWS, ROWS_AND_COLUMNS
   }

   private NamesProvided checkNames(String[] rowNames, String[] columnNames)
   {
      if (rowNames == null && columnNames != null)
         throw new IllegalArgumentException("Cannot provide column names without row names.");
      else if (rowNames != null && columnNames == null)
      {
         return checkNamesProvidedRows(rowNames);
      }
      else if (rowNames != null && columnNames != null)
      {
         return checkNamesProvidedRowsAndColumns(rowNames, columnNames);
      }
      else
      {
         return NamesProvided.NONE;
      }
   }

   private NamesProvided checkNamesProvidedRows(String[] rowNames)
   {
      if (rowNames.length != maxNumberOfRows)
         throw new IllegalArgumentException(
               "The number of row names must match the number of rows in the YoMatrix. Expected " + maxNumberOfRows + ", was " + rowNames.length);
      else if (maxNumberOfColumns > 1)
         throw new IllegalArgumentException("The YoMatrix must be a column vector if only row names are provided, else unique names cannot be generated.");
      else
         return NamesProvided.ROWS;
   }

   private NamesProvided checkNamesProvidedRowsAndColumns(String[] rowNames, String[] columnNames)
   {
      if (rowNames.length != maxNumberOfRows)
         throw new IllegalArgumentException(
               "The number of row names must match the number of rows in the YoMatrix. Expected " + maxNumberOfRows + ", was " + rowNames.length);
      else if (columnNames.length != maxNumberOfColumns)
         throw new IllegalArgumentException(
               "The number of column names must match the number of columns in the YoMatrix. Expected " + maxNumberOfColumns + ", was " + columnNames.length);
      else
         return NamesProvided.ROWS_AND_COLUMNS;
   }

   /**
    * Scale all entries of {@code this} by the provided scalar {@code scale}.
    *
    * @param scale the scalar to multiply all entries by.
    */
   public void scale(double scale)
   {
      for (int row = 0; row < getNumRows(); row++)
      {
         for (int col = 0; col < getNumCols(); col++)
         {
            unsafe_set(row, col, unsafe_get(row, col) * scale, false);
         }
      }
   }

   /**
    * Set {@code this} to {@code matrix} after it has been scaled by the provided scalar {@code scale}.
    *
    * @param scale  the scalar to multiply all of {@code matrix} by.
    * @param matrix the matrix to be scaled. Not modified.
    */
   public void scale(double scale, DMatrix matrix)
   {
      if (matrix.getNumRows() != getNumRows() || matrix.getNumCols() != getNumCols())
         throw new IllegalArgumentException(
               "Matrix dimensions do not match. Expected " + getNumRows() + "x" + getNumCols() + ", was " + matrix.getNumRows() + "x" + matrix.getNumCols());

      for (int row = 0; row < getNumRows(); row++)
      {
         for (int col = 0; col < getNumCols(); col++)
         {
            unsafe_set(row, col, matrix.unsafe_get(row, col) * scale, false);
         }
      }
   }

   /**
    * Set {@code this} to the sum of {@code a} and {@code b}.
    *
    * @param a the first matrix to be added. Not modified.
    * @param b the second matrix to be added. Not modified.
    */
   public void add(DMatrix a, DMatrix b)
   {
      add(1.0, a, 1.0, b);
   }

   /**
    * Set {@code this} to the sum of {@code a}, and {@code b} scaled by {@code beta}.
    *
    * @param a    the first matrix to be added. Not modified.
    * @param beta the scalar to multiply {@code b} by.
    * @param b    the second matrix to be added. Not modified.
    */
   public void add(DMatrix a, double beta, DMatrix b)
   {
      add(1.0, a, beta, b);
   }

   /**
    * Set {@code this} to the sum of {@code a} scaled by {@code alpha}, and {@code b} scaled by {@code beta}.
    *
    * @param alpha the scalar to multiply {@code a} by.
    * @param a     the first matrix to be added. Not modified.
    * @param beta  the scalar to multiply {@code b} by.
    * @param b     the second matrix to be added. Not modified.
    */
   public void add(double alpha, DMatrix a, double beta, DMatrix b)
   {
      if (a.getNumRows() != b.getNumRows() || a.getNumCols() != b.getNumCols())
         throw new IllegalArgumentException(
               "Matrix dimensions of A and B do not match. A: " + a.getNumRows() + "x" + a.getNumCols() + ", B: " + b.getNumRows() + "x" + b.getNumCols());

      if (a.getNumRows() != getNumRows() || a.getNumCols() != getNumCols() || b.getNumRows() != getNumRows() || b.getNumCols() != getNumCols())
         throw new IllegalArgumentException(
               "Matrix dimensions do not match. Expected " + getNumRows() + "x" + getNumCols() + ", was " + a.getNumRows() + "x" + a.getNumCols());

      for (int row = 0; row < getNumRows(); row++)
      {
         for (int col = 0; col < getNumCols(); col++)
         {
            unsafe_set(row, col, alpha * a.unsafe_get(row, col) + beta * b.unsafe_get(row, col), false);
         }
      }
   }

   /**
    * Set {@code this} to the sum of {@code this} and {@code a}.
    *
    * @param a the matrix to be added on to {@code this}. Not modified.
    */
   public void addEquals(DMatrix a)
   {
      addEquals(1.0, a);
   }

   /**
    * Set {@code this} to the sum of {@code this}, and {@code a} scaled by {@code alpha}.
    *
    * @param alpha the scalar to multiply {@code a} by.
    * @param a     the matrix to be added on to {@code this}. Not modified.
    */
   public void addEquals(double alpha, DMatrix a)
   {
      if (a.getNumRows() != getNumRows() || a.getNumCols() != getNumCols())
         throw new IllegalArgumentException(
               "Matrix dimensions do not match. Expected " + getNumRows() + "x" + getNumCols() + ", was " + a.getNumRows() + "x" + a.getNumCols());

      for (int row = 0; row < getNumRows(); row++)
      {
         for (int col = 0; col < getNumCols(); col++)
         {
            unsafe_set(row, col, unsafe_get(row, col) + alpha * a.unsafe_get(row, col), false);
         }
      }
   }

   /**
    * Get the entry of {@code this} at the specified row and column.
    *
    * @param row Matrix element's row index.
    * @param col Matrix element's column index.
    * @return the value of the matrix element at the specified row and column.
    */
   @Override
   public double get(int row, int col)
   {
      if (col < 0 || col >= getNumCols() || row < 0 || row >= getNumRows())
         throw new IllegalArgumentException("Specified element is out of bounds: (" + row + " , " + col + ")");
      return unsafe_get(row, col);
   }

   @Override
   public double unsafe_get(int row, int col)
   {
      return variables[row][col].getValue();
   }

   /**
    * Reshape the input matrix {@code matrixToPack} to match the size of {@code this} and set it to {@code this}.
    *
    * @param matrixToPack the matrix to reshape and pack {@code this} into. Modified.
    */
   public void getAndReshape(DMatrixD1 matrixToPack)
   {
      matrixToPack.reshape(getNumRows(), getNumCols());
      get(matrixToPack);
   }

   /**
    * Get {@code this} matrix and pack into {@code matrixToPack}.
    *
    * @param matrixToPack the matrix to pack {@code this} into. Modified.
    */
   public void get(DMatrix matrixToPack)
   {
      if (matrixToPack.getNumRows() != getNumRows() || matrixToPack.getNumCols() != getNumCols())
         throw new IllegalArgumentException(
               "Matrix dimensions do not match. Expected " + getNumRows() + "x" + getNumCols() + ", was " + matrixToPack.getNumRows() + "x"
               + matrixToPack.getNumCols());

      for (int row = 0; row < getNumRows(); row++)
      {
         for (int column = 0; column < getNumCols(); column++)
         {
            matrixToPack.set(row, column, unsafe_get(row, column));
         }
      }
   }

   /**
    * Set the number of rows and columns in the matrix.
    * <p>
    * The new row/column size cannot exceed the maximum row/column size. If the new size is smaller than the current size, the extra entries will be set to NaN.
    * </p>
    *
    * @param numRows The new number of rows in the matrix.
    * @param numCols The new number of columns in the matrix.
    */
   @Override
   public void reshape(int numRows, int numCols)
   {
      if (numRows > maxNumberOfRows)
         throw new IllegalArgumentException("Too many rows. Expected less or equal to " + maxNumberOfRows + ", was " + numRows);
      else if (numCols > maxNumberOfColumns)
         throw new IllegalArgumentException("Too many columns. Expected less or equal to " + maxNumberOfColumns + ", was " + numCols);
      else if (numRows < 0 || numCols < 0)
         throw new IllegalArgumentException("Cannot reshape with a negative number of rows or columns.");

      numberOfRows.set(numRows);
      numberOfColumns.set(numCols);

      for (int row = 0; row < numRows; row++)
      {
         for (int col = numCols; col < maxNumberOfColumns; col++)
         {
            unsafe_set(row, col, Double.NaN, false);
         }
      }

      for (int row = numRows; row < maxNumberOfRows; row++)
      {
         for (int col = 0; col < maxNumberOfColumns; col++)
         {
            unsafe_set(row, col, Double.NaN, false);
         }
      }
   }

   /**
    * Set the entry of {@code this} at the specified row and column to the provided value {@code val}.
    *
    * @param row Matrix element's row index.
    * @param col Matrix element's column index.
    * @param val The element's new value.
    */
   @Override
   public void set(int row, int col, double val)
   {
      if (col < 0 || col >= getNumCols() || row < 0 || row >= getNumRows())
         throw new IllegalArgumentException("Specified element is out of bounds: (" + row + " , " + col + ")");
      unsafe_set(row, col, val, false);
   }

   @Override
   public void unsafe_set(int row, int col, double val)
   {
      variables[row][col].set(val);
   }

   private void unsafe_set(int row, int col, double val, boolean notifyListeners)
   {
      variables[row][col].set(val, notifyListeners);
   }

   /**
    * Set {@code this} to the matrix {@code original}.
    * <p>
    * If {@code original} is not a {@code DMatrix}, an {@code UnsupportedOperationException} is thrown.
    * </p>
    *
    * @param original the matrix to set {@code this} to. Not modified.
    */
   @Override
   public void set(Matrix original)
   {
      if (original instanceof DMatrix otherMatrix)
      {
         reshape(otherMatrix.getNumRows(), otherMatrix.getNumCols());
         for (int row = 0; row < getNumRows(); row++)
         {
            for (int col = 0; col < getNumCols(); col++)
            {
               unsafe_set(row, col, otherMatrix.unsafe_get(row, col), false);
            }
         }
      }
      else
      {
         throw new UnsupportedOperationException("Unsupported matrix type: " + original.getClass().getSimpleName());
      }
   }

   /**
    * Set {@code this} to a matrix of size {@code numRows} by {@code numCols} with the entries all NaNs.
    *
    * @param numRows the number of rows in the matrix.
    * @param numCols the number of columns in the matrix.
    */
   public void setToNaN(int numRows, int numCols)
   {
      reshape(numRows, numCols);
      for (int row = 0; row < numRows; row++)
      {
         for (int col = 0; col < numCols; col++)
         {
            unsafe_set(row, col, Double.NaN, false);
         }
      }
   }

   /**
    * Check if {@code this} contains any NaNs.
    *
    * @return {@code true} if any of the entries of {@code this} are NaN, {@code false} otherwise.
    */
   public boolean containsNaN()
   {
      for (int row = 0; row < getNumRows(); row++)
      {
         for (int col = 0; col < getNumCols(); col++)
         {
            if (Double.isNaN(unsafe_get(row, col)))
               return true;
         }
      }
      return false;
   }

   /**
    * Set all entries of {@code this} to zero.
    */
   @Override
   public void zero()
   {
      for (int row = 0; row < maxNumberOfRows; row++)
      {
         for (int col = 0; col < maxNumberOfColumns; col++)
         {
            if (row < getNumRows() && col < getNumCols())
               unsafe_set(row, col, 0.0, false);
            else
               unsafe_set(row, col, Double.NaN, false);
         }
      }
   }

   /**
    * Set {@code this} to a matrix of size {@code numRows} by {@code numCols} with all entries set to zero.
    *
    * @param numRows the number of rows in the matrix.
    * @param numCols the number of columns in the matrix.
    */
   public void zero(int numRows, int numCols)
   {
      reshape(numRows, numCols);
      zero();
   }

   /**
    * Get the number of rows in the matrix.
    *
    * @return the number of rows in the matrix.
    */
   @Override
   public int getNumRows()
   {
      return numberOfRows.getValue();
   }

   /**
    * Get the number of columns in the matrix.
    *
    * @return the number of columns in the matrix.
    */
   @Override
   public int getNumCols()
   {
      return numberOfColumns.getValue();
   }

   /**
    * Get the number of elements in the matrix.
    *
    * @return the number of elements in the matrix.
    */
   @Override
   public int getNumElements()
   {
      return getNumRows() * getNumCols();
   }

   @Override
   public MatrixType getType()
   {
      return MatrixType.UNSPECIFIED;
   }

   @Override
   public void print()
   {
      MatrixIO.printFancy(System.out, this, MatrixIO.DEFAULT_LENGTH);
   }

   @Override
   public void print(String format)
   {
      MatrixIO.print(System.out, this, format);
   }

   @Override
   public <T extends Matrix> T createLike()
   {
      throw new UnsupportedOperationException();
   }

   @Override
   public <T extends Matrix> T create(int numRows, int numCols)
   {
      throw new UnsupportedOperationException();
   }

   @Override
   public <T extends Matrix> T copy()
   {
      throw new UnsupportedOperationException();
   }
}
