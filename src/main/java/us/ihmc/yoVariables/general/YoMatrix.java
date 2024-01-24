package us.ihmc.yoVariables.general;

import org.ejml.data.*;
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
      this(name, maxNumberOfRows, maxNumberOfColumns, null, null, registry);
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
      this(name, maxNumberOfRows, maxNumberOfColumns, rowNames, null, registry);
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
      this.maxNumberOfRows = maxNumberOfRows;
      this.maxNumberOfColumns = maxNumberOfColumns;

      this.numberOfRows = new YoInteger(name + "NumRows", registry);
      this.numberOfColumns = new YoInteger(name + "NumCols", registry);

      this.numberOfRows.set(maxNumberOfRows);
      this.numberOfColumns.set(maxNumberOfColumns);

      variables = new YoDouble[maxNumberOfRows][maxNumberOfColumns];

      if (rowNames != null && rowNames.length != maxNumberOfRows)
         throw new IllegalArgumentException("rowNames.length != maxNumberOfRows: " + rowNames.length + " != " + maxNumberOfRows);

      if (columnNames != null && columnNames.length != maxNumberOfColumns)
         throw new IllegalArgumentException("columnNames.length != maxNumberOfColumns: " + columnNames.length + " != " + maxNumberOfColumns);

      for (int row = 0; row < maxNumberOfRows; row++)
      {
         for (int column = 0; column < maxNumberOfColumns; column++)
         {
            switch (checkNames(rowNames, columnNames))
            {
               case NONE:
               {
                  variables[row][column] = new YoDouble(name + "_" + row + "_" + column, registry);  // names are simply the row and column indices
                  break;
               }
               case ROWS:
               {
                  if (maxNumberOfColumns > 1)
                     throw new IllegalArgumentException(
                           "The YoMatrix must be a column vector if only row names are provided, else unique names cannot be generated.");

                  variables[row][column] = new YoDouble(name + "_" + rowNames[row], registry);  // names are the row names, no column identifier
                  break;
               }
               case ROWS_AND_COLUMNS:
               {
                  variables[row][column] = new YoDouble(name + "_" + rowNames[row] + "_" + columnNames[column],
                                                        registry);  // names are the row and column names
                  break;
               }
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
      if (rowNames == null && columnNames == null)
         return NamesProvided.NONE;
      else if (rowNames != null && columnNames == null)
         return NamesProvided.ROWS;
      else
         return NamesProvided.ROWS_AND_COLUMNS;
   }

   public void getAndReshape(DMatrixRMaj matrixToPack)
   {
      matrixToPack.reshape(getNumRows(), getNumCols());
      get(matrixToPack);
   }

   public void get(DMatrixRMaj matrixToPack)
   {
      int numRows = matrixToPack.getNumRows();
      int numCols = matrixToPack.getNumCols();

      if (((numRows > maxNumberOfRows) || (numCols > maxNumberOfColumns)) && (numRows > 0) && (numCols > 0))
         throw new RuntimeException("Not enough rows or columns. matrixToPack is " + matrixToPack.getNumRows() + " by " + matrixToPack.getNumCols());
      if ((numRows != this.numberOfRows.getIntegerValue()) || (numCols != this.numberOfColumns.getIntegerValue()))
         throw new RuntimeException("Numer of rows and columns must be the same. Call getAndReshape() if you want to reshape the matrixToPack");

      for (int row = 0; row < numRows; row++)
      {
         for (int column = 0; column < numCols; column++)
         {
            matrixToPack.set(row, column, variables[row][column].getDoubleValue());
         }
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

   @Override
   public void set(Matrix original)
   {
      if (original instanceof DMatrix)
      {
         DMatrix otherMatrix = (DMatrix) original;
         reshape(otherMatrix.getNumRows(), otherMatrix.getNumRows());
         for (int row = 0; row < getNumRows(); row++)
         {
            for (int col = 0; col < getNumCols(); col++)
            {
               set(row, col, otherMatrix.unsafe_get(row, col));
            }
         }
      }
   }

   @Override
   public void zero()
   {
      for (int row = 0; row < getNumRows(); row++)
      {
         for (int col = 0; col < getNumCols(); col++)
         {
            variables[row][col].set(0.0);
         }
      }
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
