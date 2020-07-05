package us.ihmc.yoVariables.variable;

import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.util.YoFrameVariableNameTools;

/**
 * {@code Tuple2DBasics} abstract implementation backed with {@code YoDouble}s.
 */
public abstract class YoTuple2D implements Tuple2DBasics
{
   private final String namePrefix;
   private final String nameSuffix;

   private final YoDouble x, y;

   /**
    * Creates a new {@code YoTuple2D} using the given {@code YoVariable}s.
    *
    * @param xVariable the variable to use for the x-component.
    * @param yVariable the variable to use for the y-component.
    */
   public YoTuple2D(YoDouble xVariable, YoDouble yVariable)
   {
      namePrefix = YoFrameVariableNameTools.getCommonPrefix(xVariable.getName(), yVariable.getName());
      nameSuffix = YoFrameVariableNameTools.getCommonSuffix(xVariable.getName(), yVariable.getName());

      x = xVariable;
      y = yVariable;
   }

   /**
    * Creates a new {@code YoTuple2D}.
    *
    * @param namePrefix a unique name string to use as the prefix for child variable names.
    * @param registry   the registry to register child variables to.
    */
   public YoTuple2D(String namePrefix, YoRegistry registry)
   {
      this(namePrefix, "", registry);
   }

   /**
    * Creates a new {@code YoTuple2D}.
    *
    * @param namePrefix a unique name string to use as the prefix for child variable names.
    * @param nameSuffix a string to use as the suffix for child variable names.
    * @param registry   the registry to register child variables to.
    */
   public YoTuple2D(String namePrefix, String nameSuffix, YoRegistry registry)
   {
      this.namePrefix = namePrefix;
      this.nameSuffix = nameSuffix;

      x = new YoDouble(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), registry);
      y = new YoDouble(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), registry);
   }

   /** {@inheritDoc} */
   @Override
   public void setX(double x)
   {
      this.x.set(x);
   }

   /** {@inheritDoc} */
   @Override
   public void setY(double y)
   {
      this.y.set(y);
   }

   /** {@inheritDoc} */
   @Override
   public double getX()
   {
      return x.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getY()
   {
      return y.getDoubleValue();
   }

   /**
    * Gets the internal reference to the x-component used for this tuple.
    *
    * @return the x-component as {@code YoVariable}.
    */
   public final YoDouble getYoX()
   {
      return x;
   }

   /**
    * Gets the internal reference to the y-component used for this tuple.
    *
    * @return the y-component as {@code YoVariable}.
    */
   public final YoDouble getYoY()
   {
      return y;
   }

   /**
    * Notifies the {@code VariableChangedListener}s attached to {@code this}.
    */
   public void notifyVariableChangedListeners()
   {
      x.notifyVariableChangedListeners(); // No need to do it for all
   }

   /**
    * Attaches a listener to {@code this} that is to be triggered when this tuple components change.
    *
    * @param variableChangedListener the listener to be attached.
    */
   public final void attachVariableChangedListener(VariableChangedListener variableChangedListener)
   {
      x.addVariableChangedListener(variableChangedListener);
      y.addVariableChangedListener(variableChangedListener);
   }

   /**
    * The name prefix used at creation of this {@code this}.
    *
    * @return the name prefix {@code String}.
    */
   public String getNamePrefix()
   {
      return namePrefix;
   }

   /**
    * The name suffix used at creation of this {@code this}.
    *
    * @return the name suffix {@code String}.
    */
   public String getNameSuffix()
   {
      return nameSuffix;
   }

   /**
    * Provides a {@code String} representation of {@code this} as follows: (x, y).
    *
    * @return the {@code String} representing this tuple.
    */
   @Override
   public String toString()
   {
      return EuclidCoreIOTools.getTuple2DString(this);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object instanceof Tuple2DReadOnly)
         return equals((Tuple2DReadOnly) object);
      else
         return false;
   }

   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(getX(), getY());
   }
}