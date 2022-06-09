import React, { useCallback, useEffect, useRef, useState } from "react";

import Box from "@mui/material/Box";
import {Size} from "./types";

const DEFAULT_CONFIG = {
  sdpSemantics: "unified-plan",
};

const DEFAULT_SIZE:Size = { width: 320, height: 240}

const completeCheck = (pc: RTCPeerConnection) => {
  return new Promise<void>((resolve) => {
    if (pc.iceGatheringState === "complete") {
      resolve();
    } else {
      function checkState() {
        if (pc.iceGatheringState === "complete") {
          pc.removeEventListener("icegatheringstatechange", checkState);
          resolve();
        }
      }
      pc.addEventListener("icegatheringstatechange", checkState);
    }
  });
};
const offerRequest = (pc: RTCPeerConnection, topic: string) => {
  const offer = pc.localDescription;
  return fetch("/api/rtc/offer", {
    body: JSON.stringify({
      sdp: offer?.sdp,
      type: offer?.type,
      topic,
    }),
    headers: {
      "Content-Type": "application/json",
    },
    method: "POST",
  });
};


export type ImageViewProps = {
  topic: string;
  size?: Size;
  rtcConfig?: Record<string, any>;
};
export default function ImageView(props: ImageViewProps) {
  const { topic, size, rtcConfig } = props;

  const config: Record<string, any> =
    rtcConfig === undefined
      ? DEFAULT_CONFIG
      : { ...DEFAULT_CONFIG, ...rtcConfig };


  const _size: Size = size === undefined ? DEFAULT_SIZE: size;
  const videoRef = useRef<HTMLVideoElement>(null);

  useEffect(() => {
    if (videoRef.current) {
      const videoEl = videoRef.current;
      const pc = new RTCPeerConnection(config);

      pc.addEventListener("track", (ev) => {
        if (ev.track.kind == "video") {
          videoEl.srcObject = ev.streams[0];
        }
      });

      const negotiate = async () => {
        try {
          pc.addTransceiver("video", { direction: "recvonly" });
          const offer = await pc.createOffer();
          pc.setLocalDescription(offer);
          await completeCheck(pc);
          const response = await offerRequest(pc, topic);
          const answer = await response.json();
          pc.setRemoteDescription(answer);
        } catch (error) {
          alert(error);
        }
      };
      negotiate();

      return () => {
        if (videoRef.current) {
          setTimeout(function () {
            pc.close();
          }, 500);
        }
      };
    }
  }, [videoRef, config, topic]);

  return <video ref={videoRef} autoPlay={true} playsInline={true} 
  width={_size.width} height={_size.height} />;
}
