declare module 'urdf-loader/src/URDFDragControls.js' {
  import { Camera, Object3D } from 'three';
  import type { URDFJoint } from 'urdf-loader';

  export class URDFDragControls {
    constructor(scene: Object3D);
    enabled: boolean;
    hovered: URDFJoint | null;
    manipulating: URDFJoint | null;
    updateJoint(joint: URDFJoint, angle: number): void;
    onDragStart(joint: URDFJoint): void;
    onDragEnd(joint: URDFJoint): void;
    onHover(joint: URDFJoint): void;
    onUnhover(joint: URDFJoint): void;
    setGrabbed(grabbed: boolean): void;
  }

  export class PointerURDFDragControls extends URDFDragControls {
    constructor(scene: Object3D, camera: Camera, domElement: HTMLElement);
    dispose(): void;
  }
}
