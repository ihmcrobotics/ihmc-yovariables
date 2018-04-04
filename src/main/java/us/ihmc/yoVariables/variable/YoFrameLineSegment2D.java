package us.ihmc.yoVariables.variable;

import us.ihmc.euclid.geometry.tools.EuclidGeometryIOTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameLineSegment2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint2DBasics;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

/**
 * {@code FixedFrameLineSegment2DBasics} implementation which endpoints baked with
 * {@code YoVariable}s.
 */
public class YoFrameLineSegment2D implements FixedFrameLineSegment2DBasics
{
   private final YoFramePoint2D firstEndpoint;
   private final YoFramePoint2D secondEndpoint;

   /**
    * Creates a new {@code YoFrameLineSegment2D}.
    *
    * @param namePrefix a unique name string to use as the prefix for child variable names.
    * @param referenceFrame the reference frame for this line segment.
    * @param registry the registry to register child variables to.
    */
   public YoFrameLineSegment2D(String namePrefix, ReferenceFrame referenceFrame, YoVariableRegistry registry)
   {
      this(namePrefix, "", referenceFrame, registry);
   }

   /**
    * Creates a new {@code YoFrameLineSegment2D}.
    *
    * @param namePrefix a unique name string to use as the prefix for child variable names.
    * @param nameSuffix a string to use as the suffix for child variable names.
    * @param referenceFrame the reference frame for this line segment.
    * @param registry the registry to register child variables to.
    */
   public YoFrameLineSegment2D(String namePrefix, String nameSuffix, ReferenceFrame referenceFrame, YoVariableRegistry registry)
   {
      firstEndpoint = new YoFramePoint2D(namePrefix + "FirstEndpoint", nameSuffix, referenceFrame, registry);
      secondEndpoint = new YoFramePoint2D(namePrefix + "SecondEndpoint", nameSuffix, referenceFrame, registry);
   }

   /**
    * Creates a new {@code YoFrameLine2D} using the given {@code YoVariable}s and sets its reference
    * frame to {@code referenceFrame}.
    *
    * @param firstEndpointX the variable to use for the x-coordinate of the first endpoint.
    * @param firstEndpointY the variable to use for the y-coordinate of the first endpoint.
    * @param secondEndpointX the variable to use for the x-coordinate of the second endpoint.
    * @param secondEndpointY the variable to use for the y-coordinate of the second endpoint.
    * @param referenceFrame the reference frame for this line.
    */
   public YoFrameLineSegment2D(YoDouble firstEndpointX, YoDouble firstEndpointY, YoDouble secondEndpointX, YoDouble secondEndpointY,
                               ReferenceFrame referenceFrame)
   {
      firstEndpoint = new YoFramePoint2D(firstEndpointX, firstEndpointY, referenceFrame);
      secondEndpoint = new YoFramePoint2D(secondEndpointX, secondEndpointY, referenceFrame);
   }

   /** {@inheritDoc} */
   @Override
   public FixedFramePoint2DBasics getFirstEndpoint()
   {
      return firstEndpoint;
   }

   /** {@inheritDoc} */
   @Override
   public FixedFramePoint2DBasics getSecondEndpoint()
   {
      return secondEndpoint;
   }

   /** {@inheritDoc} */
   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return firstEndpoint.getReferenceFrame();
   }

   /**
    * Gets the internal reference to the x-coordinate used for the first endpoint of this line
    * segment.
    * 
    * @return the first endpoint x-coordinate as {@code YoVariable}.
    */
   public YoDouble getYoFirstEndpointX()
   {
      return firstEndpoint.getYoX();
   }

   /**
    * Gets the internal reference to the y-coordinate used for the first endpoint of this line
    * segment.
    * 
    * @return the first endpoint y-coordinate as {@code YoVariable}.
    */
   public YoDouble getYoFirstEndpointY()
   {
      return firstEndpoint.getYoY();
   }

   /**
    * Gets the internal reference to the x-coordinate used for the second endpoint of this line
    * segment.
    * 
    * @return the second endpoint x-coordinate as {@code YoVariable}.
    */
   public YoDouble getYoSecondEndpointX()
   {
      return secondEndpoint.getYoX();
   }

   /**
    * Gets the internal reference to the y-coordinate used for the second endpoint of this line
    * segment.
    * 
    * @return the second endpoint y-coordinate as {@code YoVariable}.
    */
   public YoDouble getYoSecondEndpointY()
   {
      return secondEndpoint.getYoY();
   }

   /**
    * Provides a {@code String} representation of this line segment as follows:<br>
    * Line segment 2D: 1st endpoint = ( 0.174, 0.732, -0.222 ), 2nd endpoint = (-0.558, -0.380,
    * 0.130 )-worldFrame
    *
    * @return the {@code String} representing this line segment.
    */
   @Override
   public String toString()
   {
      return EuclidGeometryIOTools.getLineSegment2DString(this) + "-" + getReferenceFrame();
   }
}
