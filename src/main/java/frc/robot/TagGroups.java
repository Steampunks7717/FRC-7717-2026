package frc.robot;

import java.util.HashMap;
import java.util.Map;

/**
 * Mapa de AprilTag ID → grupo de posicionamiento.
 *
 * Cada grupo tiene su propio nombre y distancia de aproximacion.
 * Cuando el robot ve un tag, GoToAprilTagCommand consulta este mapa
 * para saber a qué distancia debe posicionarse.
 *
 * Para ajustar distancias: cambiar el segundo parametro de cada TagGroup.
 */
public final class TagGroups {

  public static final class TagGroup {
    public final String name;
    public final double distanceMeters;

    public TagGroup(String name, double distanceMeters) {
      this.name = name;
      this.distanceMeters = distanceMeters;
    }
  }

  // ── Grupos RED ────────────────────────────────────────────────────────────
  //                                         nombre              distancia (m)
  public static final TagGroup RED_CLIMBER = new TagGroup("Climber Rojo",     0.5);
  public static final TagGroup RED_FRONT   = new TagGroup("Frente Rojo",      1.5);
  public static final TagGroup RED_RIGHT   = new TagGroup("Derecho Rojo",     1.5);
  public static final TagGroup RED_LEFT    = new TagGroup("Izquierdo Rojo",   1.5);
  public static final TagGroup RED_BACK    = new TagGroup("Atras Rojo",       1.5);

  // ── Grupos BLUE ───────────────────────────────────────────────────────────
  public static final TagGroup BLUE_CLIMBER = new TagGroup("Climber Azul",    0.5);
  public static final TagGroup BLUE_FRONT   = new TagGroup("Frente Azul",     1.5);
  public static final TagGroup BLUE_RIGHT   = new TagGroup("Derecho Azul",    1.5);
  public static final TagGroup BLUE_LEFT    = new TagGroup("Izquierdo Azul",  1.5);
  public static final TagGroup BLUE_BACK    = new TagGroup("Atras Azul",      1.5);

  // ── Mapa tag ID → grupo ───────────────────────────────────────────────────
  private static final Map<Integer, TagGroup> TAG_TO_GROUP = new HashMap<>();

  static {
    // Red Climber
    TAG_TO_GROUP.put(15, RED_CLIMBER);
    TAG_TO_GROUP.put(16, RED_CLIMBER);

    // Red Score Front
    TAG_TO_GROUP.put(9,  RED_FRONT);
    TAG_TO_GROUP.put(10, RED_FRONT);

    // Red Score Right-Side
    TAG_TO_GROUP.put(5,  RED_RIGHT);
    TAG_TO_GROUP.put(8,  RED_RIGHT);

    // Red Score Left-Side
    TAG_TO_GROUP.put(2,  RED_LEFT);
    TAG_TO_GROUP.put(11, RED_LEFT);

    // Red Score Back
    TAG_TO_GROUP.put(3,  RED_BACK);
    TAG_TO_GROUP.put(4,  RED_BACK);

    // Blue Climber
    TAG_TO_GROUP.put(31, BLUE_CLIMBER);
    TAG_TO_GROUP.put(32, BLUE_CLIMBER);

    // Blue Score Front
    TAG_TO_GROUP.put(25, BLUE_FRONT);
    TAG_TO_GROUP.put(26, BLUE_FRONT);

    // Blue Score Right-Side
    TAG_TO_GROUP.put(27, BLUE_RIGHT);
    TAG_TO_GROUP.put(18, BLUE_RIGHT);

    // Blue Score Left-Side
    TAG_TO_GROUP.put(21, BLUE_LEFT);
    TAG_TO_GROUP.put(24, BLUE_LEFT);

    // Blue Score Back
    TAG_TO_GROUP.put(19, BLUE_BACK);
    TAG_TO_GROUP.put(20, BLUE_BACK);
  }

  /**
   * Devuelve el TagGroup para el ID dado, o null si el tag no esta en el mapa.
   * Usar esto en GoToAprilTagCommand para determinar la distancia de aproximacion.
   */
  public static TagGroup getGroup(int tagId) {
    return TAG_TO_GROUP.get(tagId);
  }
}
