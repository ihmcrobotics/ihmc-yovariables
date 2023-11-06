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
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple4DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.yoVariables.euclid.YoQuaternion;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * {@code FixedFrameQuaternionBasics} implementation which components {@code x}, {@code y},
 * {@code z}, {@code s} are backed with {@code YoDouble}s.
 */
public class YoFrameQuaternion extends YoQuaternion implements FixedFrameQuaternionBasics
{
   private final ReferenceFrame referenceFrame;

   /**
    * Creates a new {@code YoFrameQuaternion}, initializes it to the neutral quaternion and its
    * reference frame to {@code referenceFrame}, and registers variables to {@code registry}.
    *
    * @param referenceFrame the reference frame for this {@code YoFrameQuaternion}.
    * @param namePrefix     a unique name string to use as the prefix for child variable names.
    * @param registry       the registry to register child variables to.
    */
   public YoFrameQuaternion(String namePrefix, ReferenceFrame referenceFrame, YoRegistry registry)
   {
      this(namePrefix, "", referenceFrame, registry);
   }

   /**
    * Creates a new {@code YoFrameQuaternion}, initializes it to the neutral quaternion and its
    * reference frame to {@code referenceFrame}, and registers variables to {@code registry}.
    *
    * @param namePrefix     a unique name string to use as the prefix for child variable names.
    * @param nameSuffix     a string to use as the suffix for child variable names.
    * @param referenceFrame the reference frame for this {@code YoFrameQuaternion}.
    * @param registry       the registry to register child variables to.
    */
   public YoFrameQuaternion(String namePrefix, String nameSuffix, ReferenceFrame referenceFrame, YoRegistry registry)
   {
      super(namePrefix, nameSuffix, registry);
      this.referenceFrame = referenceFrame;
      setToZero();
   }

   /**
    * Creates a new {@code YoFrameQuaternion} using the given {@code yoQuaternion}'s backing {@code YoVariable}s and sets its
    * reference frame to {@code referenceFrame}.
    *
    * @param yoQuaternion   quaternion which the backing {@code YoVariable}s are to be used for this.
    * @param referenceFrame the reference frame for this {@code YoFrameQuaternion}.
    */
   public YoFrameQuaternion(YoQuaternion yoQuaternion, ReferenceFrame referenceFrame)
   {
      super(yoQuaternion.getYoQx(), yoQuaternion.getYoQy(), yoQuaternion.getYoQz(), yoQuaternion.getYoQs());
      this.referenceFrame = referenceFrame;
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
    * @param qsVariable     an existing variable representing the s value of this
    *                       {@code YoFrameQuaternion}.
    * @param referenceFrame the reference frame for this {@code YoFrameQuaternion}.
    */
   public YoFrameQuaternion(YoDouble qxVariable, YoDouble qyVariable, YoDouble qzVariable, YoDouble qsVariable, ReferenceFrame referenceFrame)
   {
      super(qxVariable, qyVariable, qzVariable, qsVariable);
      this.referenceFrame = referenceFrame;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
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
   public YoFrameQuaternion duplicate(YoRegistry newRegistry)
   {
      YoDouble x = (YoDouble) newRegistry.findVariable(getYoQx().getFullNameString());
      YoDouble y = (YoDouble) newRegistry.findVariable(getYoQy().getFullNameString());
      YoDouble z = (YoDouble) newRegistry.findVariable(getYoQz().getFullNameString());
      YoDouble s = (YoDouble) newRegistry.findVariable(getYoQs().getFullNameString());
      return new YoFrameQuaternion(x, y, z, s, getReferenceFrame());
   }

   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(super.hashCode(), getReferenceFrame());
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
