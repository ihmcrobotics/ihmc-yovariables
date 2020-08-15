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
package us.ihmc.yoVariables.euclid;

import us.ihmc.euclid.orientation.Orientation2D;
import us.ihmc.euclid.orientation.interfaces.Orientation2DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation2DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.tools.YoGeometryNameTools;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * {@code Orientation2DBasics} implementation which position and orientation backed with
 * {@code YoVariable}s.
 */
public class YoOrientation2D implements Orientation2DBasics
{
   /** The angle in radians about the z-axis. */
   private final YoDouble yaw;

   /** Orientation used to transform {@code this} in {@link #applyTransform(Transform)}. */
   private final Orientation2D orientation = new Orientation2D();

   /**
    * Creates a new {@code YoOrientation2D}.
    *
    * @param namePrefix a unique name string to use as the prefix for child variable names.
    * @param registry   the registry to register child variables to.
    */
   public YoOrientation2D(String namePrefix, YoRegistry registry)
   {
      this(namePrefix, "", registry);
   }

   /**
    * Creates a new {@code YoOrientation2D}.
    *
    * @param namePrefix a unique name string to use as the prefix for child variable names.
    * @param nameSuffix a string to use as the suffix for child variable names.
    * @param registry   the registry to register child variables to.
    */
   public YoOrientation2D(String namePrefix, String nameSuffix, YoRegistry registry)
   {
      yaw = new YoDouble(YoGeometryNameTools.assembleName(namePrefix, "yaw", nameSuffix), registry);
   }

   /** {@inheritDoc} */
   @Override
   public void setYaw(double yaw)
   {
      this.yaw.set(yaw);
   }

   /** {@inheritDoc} */
   @Override
   public double getYaw()
   {
      return yaw.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public void applyTransform(Transform transform)
   {
      orientation.set(this);
      orientation.applyTransform(transform);
      set(orientation);
   }

   /** {@inheritDoc} */
   @Override
   public void applyInverseTransform(Transform transform)
   {
      orientation.set(this);
      orientation.applyInverseTransform(transform);
      set(orientation);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object instanceof Orientation2DReadOnly)
         return equals((Orientation2DReadOnly) object);
      else
         return false;
   }

   @Override
   public String toString()
   {
      return EuclidCoreIOTools.getOrientation2DString(this);
   }

   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(getYaw());
   }
}