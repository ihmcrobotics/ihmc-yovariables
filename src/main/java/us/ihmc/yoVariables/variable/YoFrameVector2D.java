package us.ihmc.yoVariables.variable;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector2DBasics;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

/**
 * {@code FixedFrameVector2DBasics} implementation which components {@code x}, {@code y} are baked
 * with {@code YoDouble}s.
 */
public class YoFrameVector2D extends YoFrameTuple2D implements FixedFrameVector2DBasics
{
   /**
    * Creates a new {@code YoFrameVector2D} using the given {@code YoVariable}s and sets its
    * reference frame to {@code referenceFrame}.
    *
    * @param xVariable the variable to use for the x-component.
    * @param yVariable the variable to use for the y-component.
    * @param referenceFrame the reference frame for this vector.
    */
   public YoFrameVector2D(YoDouble xVariable, YoDouble yVariable, ReferenceFrame referenceFrame)
   {
      super(xVariable, yVariable, referenceFrame);
   }

   /**
    * Creates a new {@code YoFrameVector2D}.
    *
    * @param namePrefix a unique name string to use as the prefix for child variable names.
    * @param referenceFrame the reference frame for this vector.
    * @param registry the registry to register child variables to.
    */
   public YoFrameVector2D(String namePrefix, ReferenceFrame referenceFrame, YoVariableRegistry registry)
   {
      super(namePrefix, "", referenceFrame, registry);
   }

   /**
    * Creates a new {@code YoFrameVector2D}.
    *
    * @param namePrefix a unique name string to use as the prefix for child variable names.
    * @param nameSuffix a string to use as the suffix for child variable names.
    * @param referenceFrame the reference frame for this vector.
    * @param registry the registry to register child variables to.
    */
   public YoFrameVector2D(String namePrefix, String nameSuffix, ReferenceFrame referenceFrame, YoVariableRegistry registry)
   {
      super(namePrefix, nameSuffix, referenceFrame, registry);
   }
}
