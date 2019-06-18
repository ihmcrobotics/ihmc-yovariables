package us.ihmc.yoVariables.variable;

import us.ihmc.euclid.geometry.tools.EuclidGeometryIOTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameLine2DBasics;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

/**
 * {@code FixedFrameLine2DBasics} implementation which point and line are baked with
 * {@code YoVariable}s.
 */
public class YoFrameLine2D implements FixedFrameLine2DBasics
{
   private final YoFramePoint2D point;
   private final YoFrameVector2D direction;

   /**
    * Creates a new {@code YoFrameLine2D}.
    *
    * @param namePrefix a unique name string to use as the prefix for child variable names.
    * @param referenceFrame the reference frame for this line.
    * @param registry the registry to register child variables to.
    */
   public YoFrameLine2D(String namePrefix, ReferenceFrame referenceFrame, YoVariableRegistry registry)
   {
      this(namePrefix, "", referenceFrame, registry);
   }

   /**
    * Creates a new {@code YoFrameLine2D}.
    *
    * @param namePrefix a unique name string to use as the prefix for child variable names.
    * @param nameSuffix a string to use as the suffix for child variable names.
    * @param referenceFrame the reference frame for this line.
    * @param registry the registry to register child variables to.
    */
   public YoFrameLine2D(String namePrefix, String nameSuffix, ReferenceFrame referenceFrame, YoVariableRegistry registry)
   {
      point = new YoFramePoint2D(namePrefix + "Point", nameSuffix, referenceFrame, registry);
      direction = new YoFrameVector2D(namePrefix + "Direction", nameSuffix, referenceFrame, registry);
   }

   /**
    * Creates a new {@code YoFrameLine2D} using the given {@code YoVariable}s and sets its reference
    * frame to {@code referenceFrame}.
    *
    * @param point the {@code YoFramePoint2D} to use internally for this line point.
    * @param direction the {@code YoFrameVector2D} to use internally for this line direction.
    * @throws ReferenceFrameMismatchException if {@code point} and {@code direction} are not
    *            expressed in the same reference frame.
    */
   public YoFrameLine2D(YoFramePoint2D point, YoFrameVector2D direction)
   {
      this(point.getYoX(), point.getYoY(), direction.getYoX(), direction.getYoY(), point.getReferenceFrame());
      point.checkReferenceFrameMatch(direction);
   }

   /**
    * Creates a new {@code YoFrameLine2D} using the given {@code YoVariable}s and sets its reference
    * frame to {@code referenceFrame}.
    *
    * @param pointX the variable to use for the x-coordinate of this line point.
    * @param pointY the variable to use for the y-coordinate of this line point.
    * @param directionX the variable to use for the x-component of this line direction.
    * @param directionY the variable to use for the x-component of this line direction.
    * @param referenceFrame the reference frame for this line.
    */
   public YoFrameLine2D(YoDouble pointX, YoDouble pointY, YoDouble directionX, YoDouble directionY, ReferenceFrame referenceFrame)
   {
      point = new YoFramePoint2D(pointX, pointY, referenceFrame);
      direction = new YoFrameVector2D(directionX, directionY, referenceFrame);
   }

   /** {@inheritDoc} */
   @Override
   public YoFramePoint2D getPoint()
   {
      return point;
   }

   /** {@inheritDoc} */
   @Override
   public YoFrameVector2D getDirection()
   {
      return direction;
   }

   /** {@inheritDoc} */
   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return point.getReferenceFrame();
   }

   /**
    * Gets the internal reference to the x-coordinate used for this line point.
    * 
    * @return the point x-coordinate as {@code YoVariable}.
    */
   public YoDouble getYoPointX()
   {
      return point.getYoX();
   }

   /**
    * Gets the internal reference to the y-coordinate used for this line point.
    * 
    * @return the point y-coordinate as {@code YoVariable}.
    */
   public YoDouble getYoPointY()
   {
      return point.getYoY();
   }

   /**
    * Gets the internal reference to the x-component used for this line direction.
    * 
    * @return the direction x-component as {@code YoVariable}.
    */
   public YoDouble getYoDirectionX()
   {
      return direction.getYoX();
   }

   /**
    * Gets the internal reference to the y-component used for this line direction.
    * 
    * @return the direction y-component as {@code YoVariable}.
    */
   public YoDouble getYoDirectionY()
   {
      return direction.getYoY();
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
   public YoFrameLine2D duplicate(YoVariableRegistry newRegistry)
   {
      return new YoFrameLine2D(point.duplicate(newRegistry), direction.duplicate(newRegistry));
   }

   /**
    * Provides a {@code String} representation of this line as follows:<br>
    * Line 2D: point = (x, y), direction = (x, y)-worldFrame
    *
    * @return the {@code String} representing this line.
    */
   @Override
   public String toString()
   {
      return EuclidGeometryIOTools.getLine2DString(this) + "-" + getReferenceFrame();
   }
}
