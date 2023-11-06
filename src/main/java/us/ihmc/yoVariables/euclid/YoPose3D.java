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

import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * {@code Pose3DBasics} implementation which position and orientation backed with
 * {@code YoVariable}s.
 */
public class YoPose3D implements Pose3DBasics
{
   /**
    * The position part of this pose 3D.
    */
   private final YoPoint3D position;
   /**
    * The orientation part of this pose 3D.
    */
   private final YoQuaternion orientation;

   /**
    * Creates a new {@code YoPose3D}.
    *
    * @param namePrefix a unique name string to use as the prefix for child variable names.
    * @param registry   the registry to register child variables to.
    */
   public YoPose3D(String namePrefix, YoRegistry registry)
   {
      this(namePrefix, "", registry);
   }

   /**
    * Creates a new {@code YoPose3D}.
    *
    * @param namePrefix a unique name string to use as the prefix for child variable names.
    * @param nameSuffix a string to use as the suffix for child variable names.
    * @param registry   the registry to register child variables to.
    */
   public YoPose3D(String namePrefix, String nameSuffix, YoRegistry registry)
   {
      position = new YoPoint3D(namePrefix, nameSuffix, registry);
      orientation = new YoQuaternion(namePrefix, nameSuffix, registry);
   }

   /**
    * Creates a new {@code YoPose3D} using the given {@code position} and {@code orientation}.
    *
    * @param position    the {@code YoFramePoint3D} to use internally for the position.
    * @param orientation the {@code YoFrameQuaternion} to use internally for the orientation.
    */
   public YoPose3D(YoPoint3D position, YoQuaternion orientation)
   {
      this.position = position;
      this.orientation = orientation;
   }

   @Override
   public void setX(double x)
   {
      position.setX(x);
   }

   @Override
   public void setY(double y)
   {
      position.setY(y);
   }

   @Override
   public void setZ(double z)
   {
      position.setZ(z);
   }

   @Override
   public YoPoint3D getPosition()
   {
      return position;
   }

   @Override
   public YoQuaternion getOrientation()
   {
      return orientation;
   }

   /**
    * Gets the internal reference to the x-coordinate used for the position of this pose.
    *
    * @return the position x-coordinate as {@code YoVariable}.
    */
   public YoDouble getYoX()
   {
      return position.getYoX();
   }

   /**
    * Gets the internal reference to the y-coordinate used for the position of this pose.
    *
    * @return the position y-coordinate as {@code YoVariable}.
    */
   public YoDouble getYoY()
   {
      return position.getYoY();
   }

   /**
    * Gets the internal reference to the z-coordinate used for the position of this pose.
    *
    * @return the position z-coordinate as {@code YoVariable}.
    */
   public YoDouble getYoZ()
   {
      return position.getYoZ();
   }

   /**
    * Gets the internal reference to the s-component used for the orientation of this pose.
    *
    * @return the position s-coordinate as {@code YoVariable}.
    */
   public YoDouble getYoQs()
   {
      return orientation.getYoQs();
   }

   /**
    * Gets the internal reference to the x-component used for the orientation of this pose.
    *
    * @return the position x-coordinate as {@code YoVariable}.
    */
   public YoDouble getYoQx()
   {
      return orientation.getYoQx();
   }

   /**
    * Gets the internal reference to the y-component used for the orientation of this pose.
    *
    * @return the position y-coordinate as {@code YoVariable}.
    */
   public YoDouble getYoQy()
   {
      return orientation.getYoQy();
   }

   /**
    * Gets the internal reference to the z-component used for the orientation of this pose.
    *
    * @return the position z-coordinate as {@code YoVariable}.
    */
   public YoDouble getYoQz()
   {
      return orientation.getYoQz();
   }

   @Override
   public double getX()
   {
      return position.getX();
   }

   @Override
   public double getY()
   {
      return position.getY();
   }

   @Override
   public double getZ()
   {
      return position.getZ();
   }

   @Override
   public double getYaw()
   {
      return orientation.getYaw();
   }

   @Override
   public double getPitch()
   {
      return orientation.getPitch();
   }

   @Override
   public double getRoll()
   {
      return orientation.getRoll();
   }

   @Override
   public boolean equals(Object object)
   {
      if (object instanceof Pose3DReadOnly)
         return equals((Pose3DReadOnly) object);
      else
         return false;
   }

   @Override
   public String toString()
   {
      return EuclidGeometryIOTools.getPose3DString(this);
   }

   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(position, orientation);
   }
}