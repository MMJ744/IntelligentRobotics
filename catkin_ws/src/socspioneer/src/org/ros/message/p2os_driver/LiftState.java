/* Auto-generated by genmsg_java.py for file /data/private/luser/workspace/usc-ros-pkg/p2os/p2os_driver/msg/LiftState.msg */

package org.ros.message.p2os_driver;

import java.nio.ByteBuffer;

public class LiftState extends org.ros.message.Message {

  public int state;
  public int dir;
  public float position;

  public LiftState() {
  }

  public static java.lang.String __s_getDataType() { return "p2os_driver/LiftState"; }
  @Override  public java.lang.String getDataType() { return __s_getDataType(); }
  public static java.lang.String __s_getMD5Sum() { return "4dcc2e41838611193ef6b9f90c9be41f"; }
  @Override  public java.lang.String getMD5Sum() { return __s_getMD5Sum(); }
  public static java.lang.String __s_getMessageDefinition() { return "# direction -1 is downard, +1 is upward, 0 is stationary\n" +
"int32 state\n" +
"int32 dir\n" +
"float32 position\n" +
"\n" +
""; }
  @Override  public java.lang.String getMessageDefinition() { return __s_getMessageDefinition(); }

  @Override
  public LiftState clone() {
    LiftState c = new LiftState();
    c.deserialize(serialize(0));
    return c;
  }

  @Override
  public void setTo(org.ros.message.Message m) {
    deserialize(m.serialize(0));
  }

  @Override
  public int serializationLength() {
    int __l = 0;
    __l += 4; // state
    __l += 4; // dir
    __l += 4; // position
    return __l;
  }

  @Override
  public void serialize(ByteBuffer bb, int seq) {
    bb.putInt(state);
    bb.putInt(dir);
    bb.putFloat(position);
  }

  @Override
  public void deserialize(ByteBuffer bb) {
    state = bb.getInt();
    dir = bb.getInt();
    position = bb.getFloat();
  }

  @SuppressWarnings("all")
  public boolean equals(Object o) {
    if(!(o instanceof LiftState))
      return false;
    LiftState other = (LiftState) o;
    return
      state == other.state &&
      dir == other.dir &&
      position == other.position &&
      true;
  }

  @SuppressWarnings("all")
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    long tmp;
    result = prime * result + this.state;
    result = prime * result + this.dir;
    result = prime * result + Float.floatToIntBits(this.position);
    return result;
  }
} // class LiftState