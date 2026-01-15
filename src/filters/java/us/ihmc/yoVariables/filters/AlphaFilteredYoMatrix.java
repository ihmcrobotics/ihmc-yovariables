package us.ihmc.yoVariables.filters;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.yoVariables.math.YoMatrix;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class AlphaFilteredYoMatrix extends YoMatrix
{
   private final DMatrixRMaj previous;
   private final DMatrixRMaj current;
   private final DMatrixRMaj filtered;

   private final DoubleProvider alpha;

   public AlphaFilteredYoMatrix(String name, double alpha, int numberOfRows, int numberOfColumns, String[] rowNames, String[] columnNames, YoRegistry registry)
   {
      this(name, null, alpha, numberOfRows, numberOfColumns, rowNames, columnNames, registry);
   }

   public AlphaFilteredYoMatrix(String name, DoubleProvider alpha, int numberOfRows, int numberOfColumns, String[] rowNames, String[] columnNames, YoRegistry registry)
   {
      this(name, null, alpha, numberOfRows, numberOfColumns, rowNames, columnNames, registry);
   }

   public AlphaFilteredYoMatrix(String name, String description, double alpha, int numberOfRows, int numberOfColumns, String[] rowNames, String[] columnNames, YoRegistry registry)
   {
      this(name, description, createAlpha(name, alpha, registry), numberOfRows, numberOfColumns, rowNames, columnNames, registry);
   }

   public AlphaFilteredYoMatrix(String name, String description, DoubleProvider alpha, int numberOfRows, int numberOfColumns, String[] rowNames, String[] columnNames, YoRegistry registry)
   {
      super(name, description, numberOfRows, numberOfColumns, rowNames, columnNames, registry);
      this.alpha = alpha;

      previous = new DMatrixRMaj(numberOfRows, numberOfColumns);
      current = new DMatrixRMaj(numberOfRows, numberOfColumns);
      filtered = new DMatrixRMaj(numberOfRows, numberOfColumns);
   }

   private static DoubleProvider createAlpha(String name, double value, YoRegistry registry)
   {
      YoDouble alpha = new YoDouble(name + "_alpha", registry);
      alpha.set(value);

      return alpha;
   }

   /**
    * Set the current value of the matrix to be filtered.
    * <p>
    * NOTE: This method does not solve for the filtered value. To solve for the filtered value, use {@link #solve()}.
    * </p>
    *
    * @param current the current value of the matrix to be filtered. Not modified.
    */
   @Override
   public void set(DMatrix current)
   {
      super.set(current);
      this.current.set(current);
   }

   /**
    * Assuming that the current value has been set, this method solves for the filtered value.
    * <p>
    * See {@link #set(DMatrix)} for how to set the matrix's current value.
    * </p>
    */
   public void solve()
   {
      CommonOps_DDRM.scale(alpha.getValue(), previous, filtered);

      super.get(current);
      CommonOps_DDRM.addEquals(filtered, 1 - alpha.getValue(), current);

      // Set the previous value to be the output of the filter, so it can be used next time
      previous.set(filtered);
      super.set(filtered);
   }

   /**
    * Set the current value of the matrix to be filtered and solve for the filtered value.
    *
    * @param current the current value of the matrix to be filtered. Not modified.
    */
   public void setAndSolve(DMatrix current)
   {
      CommonOps_DDRM.scale(alpha.getValue(), previous, filtered);

      super.set(current);
      this.current.set(current);
      CommonOps_DDRM.addEquals(filtered, 1 - alpha.getValue(), this.current);

      // Set the previous value to be the output of the filter, so it can be used next time
      previous.set(filtered);
      super.set(filtered);
   }
}
