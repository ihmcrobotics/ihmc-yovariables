package us.ihmc.yoVariables.variable;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple4DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.util.YoFrameVariableNameTools;

/**
 * {@code FixedFrameQuaternionBasics} implementation which components {@code x}, {@code y},
 * {@code z}, {@code s} are backed with {@code YoDouble}s.
 */
public class YoFrameQuaternion implements FixedFrameQuaternionBasics
{
   private final String namePrefix;
   private final String nameSuffix;

   private final YoDouble qx, qy, qz, qs;
   private final ReferenceFrame referenceFrame;

   /**
    * Creates a new {@code YoFrameQuaternion}, initializes it to the neutral quaternion and its
    * reference frame to {@code referenceFrame}, and registers variables to {@code registry}.
    *
    * @param referenceFrame the reference frame for this {@code YoFrameQuaternion}.
    * @param namePrefix     a unique name string to use as the prefix for child variable names.
    * @param registry       the registry to register child variables to.
    */
   public YoFrameQuaternion(String namePrefix, ReferenceFrame referenceFrame, YoVariableRegistry registry)
   {
      this(namePrefix, "", referenceFrame, registry);
   }

   /**
    * Creates a new {@code YoFrameQuaternion} using the given {@code YoVariable}s and sets its
    * reference frame to {@code referenceFrame}.
    *
    * @param namePrefix     a unique name string to use as the prefix for child variable names.
    * @param nameSuffix     a string to use as the suffix for child variable names.
    * @param referenceFrame the reference frame for this {@code YoFrameQuaternion}.
    * @param registry       the registry to register child variables to.
    */
   public YoFrameQuaternion(String namePrefix, String nameSuffix, ReferenceFrame referenceFrame, YoVariableRegistry registry)
   {
      this.namePrefix = namePrefix;
      this.nameSuffix = nameSuffix;

      qx = new YoDouble(YoFrameVariableNameTools.createQxName(namePrefix, nameSuffix), registry);
      qy = new YoDouble(YoFrameVariableNameTools.createQyName(namePrefix, nameSuffix), registry);
      qz = new YoDouble(YoFrameVariableNameTools.createQzName(namePrefix, nameSuffix), registry);
      qs = new YoDouble(YoFrameVariableNameTools.createQsName(namePrefix, nameSuffix), registry);
      this.referenceFrame = referenceFrame;

      setToZero();
   }

   /**
    * Creates a new {@code YoFrameQuaternion} using the given {@code YoVariable}s and sets its
    * reference frame to {@code referenceFrame}.
    *
    * @param qxVariable     an existing variable representing the x value of this
    *                       {@code YoFrameQuaternion}.
    * @param qyVariable     an existing variable representing the y value of this
    *                       {@code YoFrameQuaternion}.
    * @param qzVariable     an existing variable representing the z value of this
    *                       {@code YoFrameQuaternion}.
    * @param qsVariable     an existing variable representing the z value of this
    *                       {@code YoFrameQuaternion}.
    * @param referenceFrame the reference frame for this {@code YoFrameQuaternion}.
    */
   public YoFrameQuaternion(YoDouble qxVariable, YoDouble qyVariable, YoDouble qzVariable, YoDouble qsVariable, ReferenceFrame referenceFrame)
   {
      namePrefix = YoFrameVariableNameTools.getCommonPrefix(qxVariable.getName(), qyVariable.getName(), qzVariable.getName(), qsVariable.getName());
      nameSuffix = YoFrameVariableNameTools.getCommonSuffix(qxVariable.getName(), qyVariable.getName(), qzVariable.getName(), qsVariable.getName());

      qx = qxVariable;
      qy = qyVariable;
      qz = qzVariable;
      qs = qsVariable;
      this.referenceFrame = referenceFrame;
   }

   /** {@inheritDoc} */
   @Override
   public void setUnsafe(double qx, double qy, double qz, double qs)
   {
      this.qx.set(qx);
      this.qy.set(qy);
      this.qz.set(qz);
      this.qs.set(qs);
   }

   /** {@inheritDoc} */
   @Override
   public double getX()
   {
      return qx.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getY()
   {
      return qy.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getZ()
   {
      return qz.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getS()
   {
      return qs.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   /**
    * Gets the internal reference to the x-component used for this quaternion.
    *
    * @return the x-component as {@code YoVariable}.
    */
   public YoDouble getYoQx()
   {
      return qx;
   }

   /**
    * Gets the internal reference to the y-component used for this quaternion.
    *
    * @return the y-component as {@code YoVariable}.
    */
   public YoDouble getYoQy()
   {
      return qy;
   }

   /**
    * Gets the internal reference to the z-component used for this quaternion.
    *
    * @return the z-component as {@code YoVariable}.
    */
   public YoDouble getYoQz()
   {
      return qz;
   }

   /**
    * Gets the internal reference to the s-component used for this quaternion.
    *
    * @return the s-component as {@code YoVariable}.
    */
   public YoDouble getYoQs()
   {
      return qs;
   }

   /**
    * Attaches a listener to {@code this} that is to be triggered when this quaternion components
    * change.
    *
    * @param variableChangedListener the listener to be attached.
    */
   public void attachVariableChangedListener(VariableChangedListener variableChangedListener)
   {
      qx.addVariableChangedListener(variableChangedListener);
      qy.addVariableChangedListener(variableChangedListener);
      qz.addVariableChangedListener(variableChangedListener);
      qs.addVariableChangedListener(variableChangedListener);
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
    * Creates a copy of {@code this} by finding the duplicated {@code YoVariable}s in the given
    * {@link YoVariableRegistry}.
    * <p>
    * This method does not duplicate {@code YoVariable}s. Assuming the given registry is a duplicate of
    * the registry that was used to create {@code this}, this method searches for the duplicated
    * {@code YoVariable}s and use them to duplicate {@code this}.
    * </p>
    *
    * @param newRegistry YoVariableRegistry to duplicate {@code this} to.
    * @return the duplicate of {@code this}.
    */
   public YoFrameQuaternion duplicate(YoVariableRegistry newRegistry)
   {
      YoDouble x = (YoDouble) newRegistry.getYoVariable(getYoQx().getFullNameWithNameSpace());
      YoDouble y = (YoDouble) newRegistry.getYoVariable(getYoQy().getFullNameWithNameSpace());
      YoDouble z = (YoDouble) newRegistry.getYoVariable(getYoQz().getFullNameWithNameSpace());
      YoDouble s = (YoDouble) newRegistry.getYoVariable(getYoQs().getFullNameWithNameSpace());
      return new YoFrameQuaternion(x, y, z, s, getReferenceFrame());
   }

   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(EuclidHashCodeTools.toIntHashCode(getX(), getY(), getZ(), getS()), getReferenceFrame());
   }

   @Override
   public boolean equals(Object object)
   {
      if (object instanceof FrameTuple4DReadOnly)
         return equals((FrameTuple4DReadOnly) object);
      else
         return false;
   }

   /**
    * Provides a {@code String} representation of {@code this} as follows: (qx, qy, qz, qs)-worldFrame.
    *
    * @return the {@code String} representing this quaternion.
    */
   @Override
   public String toString()
   {
      return EuclidFrameIOTools.getFrameTuple4DString(this);
   }
}
