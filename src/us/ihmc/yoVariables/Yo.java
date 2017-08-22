package us.ihmc.yoVariables;

import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;

@Retention(RetentionPolicy.RUNTIME)
public @interface Yo
{
   int length() default 0;
}
