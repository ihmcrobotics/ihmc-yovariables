package us.ihmc.yoVariables.variable;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameTuple3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * {@code FixedFrameTuple3DBasics} abstract implementation backed with {@code YoDouble}s.
 */
public abstract class YoFrameTuple3D extends YoTuple3D implements FixedFrameTuple3DBasics
{
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
      super(xVariable, yVariable, zVariable);
      this.referenceFrame = referenceFrame;
   }

   /**
    * Creates a new {@code YoFrameTuple3D}.
    *
    * @param namePrefix     a unique name string to use as the prefix for child variable names.
    * @param referenceFrame the reference frame for this tuple.
    * @param registry       the registry to register child variables to.
    */
   public YoFrameTuple3D(String namePrefix, ReferenceFrame referenceFrame, YoRegistry registry)
   {
      super(namePrefix, registry);
      this.referenceFrame = referenceFrame;
   }

   /**
    * Creates a new {@code YoFrameTuple3D}.
    *
    * @param namePrefix     a unique name string to use as the prefix for child variable names.
    * @param nameSuffix     a string to use as the suffix for child variable names.
    * @param referenceFrame the reference frame for this tuple.
    * @param registry       the registry to register child variables to.
    */
   public YoFrameTuple3D(String namePrefix, String nameSuffix, ReferenceFrame referenceFrame, YoRegistry registry)
   {
      super(namePrefix, nameSuffix, registry);
      this.referenceFrame = referenceFrame;
   }

   /** {@inheritDoc} */
   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   /**
    * Provides a {@code String} representation of {@code this} as follows: (x, y, z)-worldFrame.
    *
    * @return the {@code String} representing this tuple.
    */
   @Override
   public String toString()
   {
      return EuclidFrameIOTools.getFrameTuple3DString(this);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object instanceof FrameTuple3DReadOnly)
         return equals((FrameTuple3DReadOnly) object);
      else
         return false;
   }

   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(EuclidHashCodeTools.toIntHashCode(getX(), getY(), getZ()), getReferenceFrame());
   }
}