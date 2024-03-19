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
               case NONE -> variables[row][column] = new YoDouble(name + "_" + row + "_" + column, description, registry);
               case ROWS -> variables[row][column] = new YoDouble(name + rowNames[row], description, registry);
               case ROWS_AND_COLUMNS -> variables[row][column] = new YoDouble(name + rowNames[row] + columnNames[column], description, registry);
            }
         }
      }
   }

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

   public void scale(double scale)
   {
      for (int row = 0; row < getNumRows(); row++)
      {
         for (int col = 0; col < getNumCols(); col++)
         {
            unsafe_set(row, col, unsafe_get(row, col) * scale);
         }
      }
   }

   public void scale(double scale, DMatrix matrix)
   {
      if (matrix.getNumRows() != getNumRows() || matrix.getNumCols() != getNumCols())
         throw new IllegalArgumentException(
               "Matrix dimensions do not match. Expected " + getNumRows() + "x" + getNumCols() + ", was " + matrix.getNumRows() + "x" + matrix.getNumCols());

      for (int row = 0; row < getNumRows(); row++)
      {
         for (int col = 0; col < getNumCols(); col++)
         {
            unsafe_set(row, col, matrix.unsafe_get(row, col) * scale);
         }
      }
   }

   public void add(DMatrix a, DMatrix b)
   {
      add(1.0, a, 1.0, b);
   }

   public void add(DMatrix a, double beta, DMatrix b)
   {
      add(1.0, a, beta, b);
   }

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
            unsafe_set(row, col, alpha * a.unsafe_get(row, col) + beta * b.unsafe_get(row, col));
         }
      }
   }

   public void addEquals(DMatrix a)
   {
      addEquals(1.0, a);
   }

   public void addEquals(double alpha, DMatrix a)
   {
      if (a.getNumRows() != getNumRows() || a.getNumCols() != getNumCols())
         throw new IllegalArgumentException(
               "Matrix dimensions do not match. Expected " + getNumRows() + "x" + getNumCols() + ", was " + a.getNumRows() + "x" + a.getNumCols());

      for (int row = 0; row < getNumRows(); row++)
      {
         for (int col = 0; col < getNumCols(); col++)
         {
            unsafe_set(row, col, unsafe_get(row, col) + alpha * a.unsafe_get(row, col));
         }
      }
   }

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

   public void getAndReshape(DMatrixD1 matrixToPack)
   {
      matrixToPack.reshape(getNumRows(), getNumCols());
      get(matrixToPack);
   }

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
            unsafe_set(row, col, Double.NaN);
         }
      }

      for (int row = numRows; row < maxNumberOfRows; row++)
      {
         for (int col = 0; col < maxNumberOfColumns; col++)
         {
            unsafe_set(row, col, Double.NaN);
         }
      }
   }

   @Override
   public void set(int row, int col, double val)
   {
      if (col < 0 || col >= getNumCols() || row < 0 || row >= getNumRows())
         throw new IllegalArgumentException("Specified element is out of bounds: (" + row + " , " + col + ")");
      unsafe_set(row, col, val);
   }

   @Override
   public void unsafe_set(int row, int col, double val)
   {
      variables[row][col].set(val);
   }

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
               unsafe_set(row, col, otherMatrix.unsafe_get(row, col));
            }
         }
      }
      else
      {
         throw new UnsupportedOperationException("Unsupported matrix type: " + original.getClass().getSimpleName());
      }
   }

   public void setToNaN(int numRows, int numCols)
   {
      reshape(numRows, numCols);
      for (int row = 0; row < numRows; row++)
      {
         for (int col = 0; col < numCols; col++)
         {
            unsafe_set(row, col, Double.NaN);
         }
      }
   }

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

   @Override
   public void zero()
   {
      for (int row = 0; row < maxNumberOfRows; row++)
      {
         for (int col = 0; col < maxNumberOfColumns; col++)
         {
            if (row < getNumRows() && col < getNumCols())
               unsafe_set(row, col, 0.0);
            else
               unsafe_set(row, col, Double.NaN);
         }
      }
   }

   public void zero(int numRows, int numCols)
   {
      reshape(numRows, numCols);
      zero();
   }

   @Override
   public int getNumRows()
   {
      return numberOfRows.getValue();
   }

   @Override
   public int getNumCols()
   {
      return numberOfColumns.getValue();
   }

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
