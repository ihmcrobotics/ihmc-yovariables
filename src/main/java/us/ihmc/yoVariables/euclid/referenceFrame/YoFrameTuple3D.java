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
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameTuple3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.yoVariables.euclid.YoTuple3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

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

   /**
    * {@inheritDoc}
    */
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