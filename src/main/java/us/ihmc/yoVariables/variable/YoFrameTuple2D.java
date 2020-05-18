package us.ihmc.yoVariables.variable;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameTuple2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

/**
 * {@code FixedFrameTuple2DBasics} abstract implementation backed with {@code YoDouble}s.
 */
public abstract class YoFrameTuple2D extends YoTuple2D implements FixedFrameTuple2DBasics
{
   private final ReferenceFrame referenceFrame;

   /**
    * Creates a new {@code YoFrameTuple2D} using the given {@code YoVariable}s and sets its reference
    * frame to {@code referenceFrame}.
    *
    * @param xVariable      the variable to use for the x-component.
    * @param yVariable      the variable to use for the y-component.
    * @param referenceFrame the reference frame for this tuple.
    */
   public YoFrameTuple2D(YoDouble xVariable, YoDouble yVariable, ReferenceFrame referenceFrame)
   {
      super(xVariable, yVariable);
      this.referenceFrame = referenceFrame;
   }

   /**
    * Creates a new {@code YoFrameTuple2D}.
    *
    * @param namePrefix     a unique name string to use as the prefix for child variable names.
    * @param referenceFrame the reference frame for this tuple.
    * @param registry       the registry to register child variables to.
    */
   public YoFrameTuple2D(String namePrefix, ReferenceFrame referenceFrame, YoVariableRegistry registry)
   {
      super(namePrefix, registry);
      this.referenceFrame = referenceFrame;
   }

   /**
    * Creates a new {@code YoFrameTuple2D}.
    *
    * @param namePrefix     a unique name string to use as the prefix for child variable names.
    * @param nameSuffix     a string to use as the suffix for child variable names.
    * @param referenceFrame the reference frame for this tuple.
    * @param registry       the registry to register child variables to.
    */
   public YoFrameTuple2D(String namePrefix, String nameSuffix, ReferenceFrame referenceFrame, YoVariableRegistry registry)
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
    * Provides a {@code String} representation of {@code this} as follows: (x, y)-worldFrame.
    *
    * @return the {@code String} representing this tuple.
    */
   @Override
   public String toString()
   {
      return EuclidFrameIOTools.getFrameTuple2DString(this);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object instanceof FrameTuple2DReadOnly)
         return equals((FrameTuple2DReadOnly) object);
      else
         return false;
   }

   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(EuclidHashCodeTools.toIntHashCode(getX(), getY()), getReferenceFrame());
   }
}