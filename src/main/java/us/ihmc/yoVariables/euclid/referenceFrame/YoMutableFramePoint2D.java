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
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.interfaces.FrameIndexMap;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

/**
 * {@code FramePoint2DBasics} implementation which coordinates {@code x}, {@code y} are backed with
 * {@code YoDouble}s.
 */
public class YoMutableFramePoint2D extends YoMutableFrameTuple2D implements FramePoint2DBasics
{
   /**
    * Creates a new {@code YoMutableFramePoint2D}, initializes its coordinates to zero and its
    * reference frame to {@code ReferenceFrame.getWorldFrame()}.
    *
    * @param namePrefix a unique name string to use as the prefix for child variable names.
    * @param nameSuffix a string to use as the suffix for child variable names.
    * @param registry   the registry to register child variables to.
    */
   public YoMutableFramePoint2D(String namePrefix, String nameSuffix, YoRegistry registry)
   {
      super(namePrefix, nameSuffix, registry);
   }

   /**
    * Creates a new {@code YoMutableFramePoint2D}, initializes its coordinates to zero and its
    * reference frame to {@code referenceFrame}.
    *
    * @param namePrefix     a unique name string to use as the prefix for child variable names.
    * @param nameSuffix     a string to use as the suffix for child variable names.
    * @param referenceFrame the reference frame for this point.
    * @param registry       the registry to register child variables to.
    */
   public YoMutableFramePoint2D(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame)
   {
      super(namePrefix, nameSuffix, registry, referenceFrame);
   }

   /**
    * Creates a new {@code YoMutableFramePoint2D}, initializes its coordinates and its reference frame.
    *
    * @param namePrefix     a unique name string to use as the prefix for child variable names.
    * @param nameSuffix     a string to use as the suffix for child variable names.
    * @param referenceFrame the reference frame for this point.
    * @param registry       the registry to register child variables to.
    * @param x              the initial value for the x-coordinate.
    * @param y              the initial value for the y-coordinate.
    */
   public YoMutableFramePoint2D(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame, double x, double y)
   {
      super(namePrefix, nameSuffix, registry, referenceFrame, x, y);
   }

   /**
    * Creates a new {@code YoMutableFramePoint2D}, initializes its coordinates and its reference frame.
    *
    * @param namePrefix     a unique name string to use as the prefix for child variable names.
    * @param nameSuffix     a string to use as the suffix for child variable names.
    * @param referenceFrame the reference frame for this point.
    * @param registry       the registry to register child variables to.
    * @param tupleArray     the array containing this point's coordinates. Not modified.
    */
   public YoMutableFramePoint2D(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame, double[] tupleArray)
   {
      super(namePrefix, nameSuffix, registry, referenceFrame, tupleArray);
   }

   /**
    * Creates a new {@code YoMutableFramePoint2D}, initializes its coordinates and its reference frame.
    *
    * @param namePrefix      a unique name string to use as the prefix for child variable names.
    * @param nameSuffix      a string to use as the suffix for child variable names.
    * @param referenceFrame  the reference frame for this point.
    * @param registry        the registry to register child variables to.
    * @param tuple2DReadOnly the tuple used to initializes this point's coordinates. Not modified.
    */
   public YoMutableFramePoint2D(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame, Tuple2DReadOnly tuple2DReadOnly)
   {
      super(namePrefix, nameSuffix, registry, referenceFrame, tuple2DReadOnly);
   }

   /**
    * Creates a new {@code YoMutableFramePoint2D}, initializes its coordinates and its reference frame.
    *
    * @param namePrefix      a unique name string to use as the prefix for child variable names.
    * @param nameSuffix      a string to use as the suffix for child variable names.
    * @param referenceFrame  the reference frame for this point.
    * @param registry        the registry to register child variables to.
    * @param tuple3DReadOnly the tuple used to initializes this point's coordinates. Not modified.
    */
   public YoMutableFramePoint2D(String namePrefix, String nameSuffix, YoRegistry registry, ReferenceFrame referenceFrame, Tuple3DReadOnly tuple3DReadOnly)
   {
      super(namePrefix, nameSuffix, registry, referenceFrame, tuple3DReadOnly);
   }

   /**
    * Creates a new {@code YoMutableFramePoint2D}, initializes its coordinates and its reference frame.
    *
    * @param namePrefix           a unique name string to use as the prefix for child variable names.
    * @param nameSuffix           a string to use as the suffix for child variable names.
    * @param registry             the registry to register child variables to.
    * @param frameTuple2DReadOnly the tuple used to initializes this point's coordinates and reference
    *                             frame. Not modified.
    */
   public YoMutableFramePoint2D(String namePrefix, String nameSuffix, YoRegistry registry, FrameTuple2DReadOnly frameTuple2DReadOnly)
   {
      super(namePrefix, nameSuffix, registry, frameTuple2DReadOnly);
   }

   /**
    * Creates a new {@code YoMutableFramePoint2D}, initializes its coordinates and its reference frame.
    *
    * @param namePrefix           a unique name string to use as the prefix for child variable names.
    * @param nameSuffix           a string to use as the suffix for child variable names.
    * @param registry             the registry to register child variables to.
    * @param frameTuple3DReadOnly the tuple used to initializes this point's coordinates and reference
    *                             frame. Not modified.
    */
   public YoMutableFramePoint2D(String namePrefix, String nameSuffix, YoRegistry registry, FrameTuple3DReadOnly frameTuple3DReadOnly)
   {
      super(namePrefix, nameSuffix, registry, frameTuple3DReadOnly);
   }

   /**
    * Creates a new {@code YoMutableFramePoint2D} using the given {@code YoVariable}s and sets its
    * reference frame to {@code referenceFrame}.
    *
    * @param x             the variable to use for the x-coordinate.
    * @param y             the variable to use for the y-coordinate.
    * @param frameIndex    the variable used to track the current reference frame.
    * @param frameIndexMap the frame index manager used to store and retrieve a reference frame.
    */
   public YoMutableFramePoint2D(YoDouble x, YoDouble y, YoLong frameIndex, FrameIndexMap frameIndexMap)
   {
      super(x, y, frameIndex, frameIndexMap);
   }

   @Override
   public String toString()
   {
      return super.toString() + " (point)";
   }
}
