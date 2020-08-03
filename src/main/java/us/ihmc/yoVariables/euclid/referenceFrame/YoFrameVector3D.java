/*
 * Copyright 2020 Florida Institute for Human and Machine Cognition (IHMC)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package us.ihmc.yoVariables.euclid.referenceFrame;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * {@code FixedFrameVector3DBasics} implementation which components {@code x}, {@code y}, {@code z}
 * are backed with {@code YoDouble}s.
 */
public class YoFrameVector3D extends YoFrameTuple3D implements FixedFrameVector3DBasics
{
   /**
    * Creates a new {@code YoFrameVector3D} using the given {@code YoVariable}s and sets its reference
    * frame to {@code referenceFrame}.
    *
    * @param xVariable      the variable to use for the x-component.
    * @param yVariable      the variable to use for the y-component.
    * @param zVariable      the variable to use for the z-component.
    * @param referenceFrame the reference frame for this vector.
    */
   public YoFrameVector3D(YoDouble xVariable, YoDouble yVariable, YoDouble zVariable, ReferenceFrame referenceFrame)
   {
      super(xVariable, yVariable, zVariable, referenceFrame);
   }

   /**
    * Creates a new {@code YoFrameVector3D}.
    *
    * @param namePrefix     a unique name string to use as the prefix for child variable names.
    * @param referenceFrame the reference frame for this vector.
    * @param registry       the registry to register child variables to.
    */
   public YoFrameVector3D(String namePrefix, ReferenceFrame referenceFrame, YoRegistry registry)
   {
      super(namePrefix, "", referenceFrame, registry);
   }

   /**
    * Creates a new {@code YoFrameVector3D}.
    *
    * @param namePrefix     a unique name string to use as the prefix for child variable names.
    * @param nameSuffix     a string to use as the suffix for child variable names.
    * @param referenceFrame the reference frame for this vector.
    * @param registry       the registry to register child variables to.
    */
   public YoFrameVector3D(String namePrefix, String nameSuffix, ReferenceFrame referenceFrame, YoRegistry registry)
   {
      super(namePrefix, nameSuffix, referenceFrame, registry);
   }

   /**
    * Creates a copy of {@code this} by finding the duplicated {@code YoVariable}s in the given
    * {@link YoRegistry}.
    * <p>
    * This method does not duplicate {@code YoVariable}s. Assuming the given registry is a duplicate of
    * the registry that was used to create {@code this}, this method searches for the duplicated
    * {@code YoVariable}s and use them to duplicate {@code this}.
    * </p>
    *
    * @param newRegistry YoRegistry to duplicate {@code this} to.
    * @return the duplicate of {@code this}.
    */
   public YoFrameVector3D duplicate(YoRegistry newRegistry)
   {
      YoDouble x = (YoDouble) newRegistry.findVariable(getYoX().getFullNameString());
      YoDouble y = (YoDouble) newRegistry.findVariable(getYoY().getFullNameString());
      YoDouble z = (YoDouble) newRegistry.findVariable(getYoZ().getFullNameString());
      return new YoFrameVector3D(x, y, z, getReferenceFrame());
   }
}
