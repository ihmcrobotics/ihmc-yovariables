package us.ihmc.yoVariables.variable;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameTuple3DBasics;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.util.YoFrameVariableNameTools;

/**
 * {@code FixedFrameTuple3DBasics} abstract implementation backed with {@code YoDouble}s.
 */
public abstract class YoFrameTuple3D implements FixedFrameTuple3DBasics
{
   private final String namePrefix;
   private final String nameSuffix;

   private final YoDouble x, y, z;
   private final ReferenceFrame referenceFrame;

   /**
    * Creates a new {@code YoFrameTuple3D} using the given {@code YoVariable}s and sets its reference
    * frame to {@code referenceFrame}.
    *
    * @param xVariable      the variable to use for the x-component.
    * @param yVariable      the variable to use for the y-component.
    * @param zVariable      the variable to use for the z-component.
    * @param referenceFrame the reference frame for this tuple.
    */
   public YoFrameTuple3D(YoDouble xVariable, YoDouble yVariable, YoDouble zVariable, ReferenceFrame referenceFrame)
   {
      namePrefix = YoFrameVariableNameTools.getCommonPrefix(xVariable.getName(), yVariable.getName(), zVariable.getName());
      nameSuffix = YoFrameVariableNameTools.getCommonSuffix(xVariable.getName(), yVariable.getName(), zVariable.getName());

      x = xVariable;
      y = yVariable;
      z = zVariable;
      this.referenceFrame = referenceFrame;
   }

   /**
    * Creates a new {@code YoFrameTuple3D}.
    *
    * @param namePrefix     a unique name string to use as the prefix for child variable names.
    * @param referenceFrame the reference frame for this tuple.
    * @param registry       the registry to register child variables to.
    */
   public YoFrameTuple3D(String namePrefix, ReferenceFrame referenceFrame, YoVariableRegistry registry)
   {
      this(namePrefix, "", referenceFrame, registry);
   }

   /**
    * Creates a new {@code YoFrameTuple3D}.
    *
    * @param namePrefix     a unique name string to use as the prefix for child variable names.
    * @param nameSuffix     a string to use as the suffix for child variable names.
    * @param referenceFrame the reference frame for this tuple.
    * @param registry       the registry to register child variables to.
    */
   public YoFrameTuple3D(String namePrefix, String nameSuffix, ReferenceFrame referenceFrame, YoVariableRegistry registry)
   {
      this.namePrefix = namePrefix;
      this.nameSuffix = nameSuffix;

      x = new YoDouble(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), registry);
      y = new YoDouble(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), registry);
      z = new YoDouble(YoFrameVariableNameTools.createZName(namePrefix, nameSuffix), registry);
      this.referenceFrame = referenceFrame;
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
   public void setZ(double z)
   {
      this.z.set(z);
   }

   /** {@inheritDoc} */
   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
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

   /** {@inheritDoc} */
   @Override
   public double getZ()
   {
      return z.getDoubleValue();
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
    * Gets the internal reference to the z-component used for this tuple.
    *
    * @return the z-component as {@code YoVariable}.
    */
   public final YoDouble getYoZ()
   {
      return z;
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
      z.addVariableChangedListener(variableChangedListener);
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
    * Provides a {@code String} representation of {@code this} as follows: (x, y, z)-worldFrame.
    *
    * @return the {@code String} representing this tuple.
    */
   @Override
   public String toString()
   {
      return EuclidCoreIOTools.getTuple3DString(this) + "-" + referenceFrame;
   }
}