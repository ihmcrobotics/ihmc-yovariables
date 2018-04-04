package us.ihmc.yoVariables.variable;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

/**
 * {@code FixedFramePoint3DBasics} implementation which coordinates {@code x}, {@code y}, {@code z}
 * are baked with {@code YoDouble}s.
 */
public class YoFramePoint3D extends YoFrameTuple3D implements FixedFramePoint3DBasics
{
   /**
    * Creates a new {@code YoFramePoint3D} using the given {@code YoVariable}s and sets its
    * reference frame to {@code referenceFrame}.
    *
    * @param xVariable the variable to use for the x-coordinate.
    * @param yVariable the variable to use for the y-coordinate.
    * @param zVariable the variable to use for the z-coordinate.
    * @param referenceFrame the reference frame for this point.
    */
   public YoFramePoint3D(YoDouble xVariable, YoDouble yVariable, YoDouble zVariable, ReferenceFrame referenceFrame)
   {
      super(xVariable, yVariable, zVariable, referenceFrame);
   }

   /**
    * Creates a new {@code YoFramePoint3D}.
    *
    * @param namePrefix a unique name string to use as the prefix for child variable names.
    * @param referenceFrame the reference frame for this point.
    * @param registry the registry to register child variables to.
    */
   public YoFramePoint3D(String namePrefix, ReferenceFrame referenceFrame, YoVariableRegistry registry)
   {
      super(namePrefix, "", referenceFrame, registry);
   }

   /**
    * Creates a new {@code YoFramePoint3D}.
    *
    * @param namePrefix a unique name string to use as the prefix for child variable names.
    * @param nameSuffix a string to use as the suffix for child variable names.
    * @param referenceFrame the reference frame for this point.
    * @param registry the registry to register child variables to.
    */
   public YoFramePoint3D(String namePrefix, String nameSuffix, ReferenceFrame referenceFrame, YoVariableRegistry registry)
   {
      super(namePrefix, nameSuffix, referenceFrame, registry);
   }
}
