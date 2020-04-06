package us.ihmc.yoVariables.variable;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

/**
 * {@code FixedFramePoint3DBasics} implementation which coordinates {@code x}, {@code y}, {@code z}
 * are backed with {@code YoDouble}s.
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

   /**
    * Creates a copy of {@code this} by finding the duplicated {@code YoVariable}s in the given
    * {@link YoVariableRegistry}.
    * <p>
    * This method does not duplicate {@code YoVariable}s. Assuming the given registry is a duplicate
    * of the registry that was used to create {@code this}, this method searches for the duplicated
    * {@code YoVariable}s and use them to duplicate {@code this}.
    * </p>
    *
    * @param newRegistry YoVariableRegistry to duplicate {@code this} to.
    * @return the duplicate of {@code this}.
    */
   public YoFramePoint3D duplicate(YoVariableRegistry newRegistry)
   {
      YoDouble x = (YoDouble) newRegistry.getVariable(getYoX().getFullNameWithNameSpace());
      YoDouble y = (YoDouble) newRegistry.getVariable(getYoY().getFullNameWithNameSpace());
      YoDouble z = (YoDouble) newRegistry.getVariable(getYoZ().getFullNameWithNameSpace());
      return new YoFramePoint3D(x, y, z, getReferenceFrame());
   }
}
