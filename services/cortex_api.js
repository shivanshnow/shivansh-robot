/**
 * ============================================================================
 * 🧠 S2-R2-D2 CLOUD CORTEX & OUTBOUND API SERVICE GATEWAY
 * ============================================================================
 * All external outbound network requests are strictly isolated in this module.
 * ZERO telemetry tracking • ZERO PII storage • 100% Client-Side Private
 */

const CortexAPI = (function() {
  'use strict';

  const STORAGE_KEY = 's2r2d2_gemini_api_key';

  function getApiKey() {
    try {
      return localStorage.getItem(STORAGE_KEY) || '';
    } catch (e) {
      return '';
    }
  }

  function setApiKey(key) {
    try {
      const clean = (key || '').trim();
      if (clean.length > 0) {
        localStorage.setItem(STORAGE_KEY, clean);
      } else {
        localStorage.removeItem(STORAGE_KEY);
      }
    } catch (e) {
      console.error('[CortexAPI] Failed to update API key storage:', e);
    }
  }

  function hasApiKey() {
    const key = getApiKey();
    return key && key.length > 10;
  }

  /**
   * 🌤️ Open-Meteo Free Weather API (Zero auth required)
   */
  async function fetchLiveWeather(lat, lon) {
    try {
      const res = await fetch(`https://api.open-meteo.com/v1/forecast?latitude=${lat}&longitude=${lon}&current_weather=true`);
      if (!res.ok) throw new Error(`Weather HTTP ${res.status}`);
      const data = await res.json();
      if (data && data.current_weather) {
        const temp = Math.round(data.current_weather.temperature);
        const code = data.current_weather.weathercode;
        let condition = "Sunny";
        if (code >= 1 && code <= 3) condition = "Cloudy";
        else if (code >= 45 && code <= 48) condition = "Foggy";
        else if (code >= 51 && code <= 67) condition = "Rainy";
        else if (code >= 71 && code <= 77) condition = "Snowy";
        else if (code >= 80 && code <= 82) condition = "Showers";
        else if (code >= 95) condition = "Stormy";
        return { temp: `${temp}°C`, condition: condition, raw: data.current_weather };
      }
    } catch (err) {
      console.warn("[CortexAPI] Weather fetch warning:", err.message);
    }
    return { temp: "--°C", condition: "Clear", raw: null };
  }

  /**
   * 👑 Royal Executive Briefing (Gemini 2.5 Flash)
   */
  async function fetchRoyalBriefing(pilot, timeStr, temp, condition) {
    const defaultBriefing = `Good day ${pilot}! It is currently ${timeStr}, ${temp} with ${condition}. S2-R2-D2 reports all perimeter defenses and royal motors are fully armed and at your command!`;
    const apiKey = getApiKey();
    if (!apiKey || apiKey.length < 10) {
      return defaultBriefing;
    }

    try {
      const prompt = `You are S2-R2-D2, an authentic, witty, loyal Star Wars Astromech Droid and Grandmaster Second to ${pilot}.
Generate a concise, energetic, humorous 2-sentence executive status briefing for ${pilot}.
Context: Current Time is ${timeStr}, Weather is ${temp} (${condition}).
Sign off with a crisp astromech tactical line!`;

      const resp = await fetch(`https://generativelanguage.googleapis.com/v1beta/models/gemini-2.5-flash:generateContent?key=${apiKey}`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ contents: [{ parts: [{ text: prompt }] }] })
      });
      const d = await resp.json();
      if (d.error) throw new Error(d.error.message || "API Error");
      const genText = d.candidates?.[0]?.content?.parts?.[0]?.text;
      if (genText) return genText.replace(/[*#]/g, '').trim();
    } catch (e) {
      console.warn("[CortexAPI] Gemini Briefing fallback:", e.message);
    }
    return defaultBriefing;
  }

  /**
   * 🧠 Vocal Brain / Ask S2-R2-D2 Anything (Gemini 2.5 Flash)
   */
  async function askVocalBrain(pilot, query) {
    const apiKey = getApiKey();
    if (!apiKey || apiKey.length < 10) {
      const offlineReplies = [
        `Astromech offline circuits humming! Standing by for ${pilot}'s manual command!`,
        `Local radar active! Tap the badge above if you'd like to link my cloud intelligence!`,
        `BEEP-BOOP! Sensor telemetry nominal at 100% efficiency!`
      ];
      return {
        text: offlineReplies[Math.floor(Math.random() * offlineReplies.length)],
        isOffline: true
      };
    }

    try {
      const prompt = `You are S2-R2-D2, an intelligent and witty Astromech Droid companion to ${pilot}.
Answer this question concisely in 1 to 2 crisp, smart sentences with subtle astromech personality: "${query}"`;

      const resp = await fetch(`https://generativelanguage.googleapis.com/v1beta/models/gemini-2.5-flash:generateContent?key=${apiKey}`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ contents: [{ parts: [{ text: prompt }] }] })
      });
      const d = await resp.json();
      if (d.error) throw new Error(d.error.message || "API Error");
      const reply = d.candidates?.[0]?.content?.parts?.[0]?.text || "Astromech circuits humming affirmative!";
      return {
        text: reply.replace(/[*#]/g, '').trim(),
        isOffline: false
      };
    } catch (err) {
      console.error("[CortexAPI] Vocal Error:", err);
      throw err;
    }
  }

  /**
   * 🎯 Spatial Vision Object Detection (Gemini Robotics ER-2)
   */
  async function analyzeVisionTarget(goal, base64Image) {
    const apiKey = getApiKey();
    if (!apiKey || apiKey.length < 10) {
      throw new Error("Gemini API key not configured. Tap the badge to set your key.");
    }

    const endpoint = `https://generativelanguage.googleapis.com/v1beta/models/gemini-robotics-er-2-preview:generateContent?key=${apiKey}`;
    const promptText = `You are the spatial visual cortex of a differential-drive robot named S2-R2-D2.
Given this camera image and the user's goal: "${goal}", analyze the scene, locate the target object relative to the robot's front bumper, and return ONLY a JSON object with this exact schema:
{
  "target": "name of target object",
  "bearing_deg": number (angle in degrees relative to center: negative is left, positive is right),
  "distance_cm": number (approx distance in cm),
  "reasoning": "1 sentence visual observation",
  "actions": [
    {"cmd": "F"|"B"|"L"|"R"|"S", "duration_ms": number}
  ]
}`;

    const response = await fetch(endpoint, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        contents: [{
          parts: [
            { text: promptText },
            { inline_data: { mime_type: "image/jpeg", data: base64Image } }
          ]
        }]
      })
    });

    if (!response.ok) {
      const errData = await response.json();
      throw new Error(errData.error?.message || "HTTP " + response.status);
    }

    const data = await response.json();
    let rawText = data.candidates?.[0]?.content?.parts?.[0]?.text || "{}";
    rawText = rawText.replace(/```json/g, '').replace(/```/g, '').trim();
    return JSON.parse(rawText);
  }

  /**
   * ♟️ Physical Chessboard Analysis (Gemini 2.5 Flash)
   */
  async function analyzeChessboard(base64Image) {
    const apiKey = getApiKey();
    if (!apiKey || apiKey.length < 10) {
      throw new Error("Gemini API key not configured. Tap the badge to set your key.");
    }

    const promptText = `You are a World Chess Grandmaster's Tactical Second. Look at the real physical chessboard in this image.
1. Identify the board position and active piece structure.
2. Determine the single strongest, sharpest candidate move for the active side (e.g. "Nf3+", "Qxd5", "e4", "Bxf7+").
3. Estimate the centipawn evaluation (e.g. "+2.40", "-1.50", "0.00").
4. Explain the key tactical idea in 1 sharp, inspiring sentence.

Return ONLY a valid JSON object matching this schema with NO markdown code blocks:
{
  "best_move": "Nf3+",
  "evaluation": "+2.40",
  "tactical_idea": "Knight forks the King on e8 and Rook on a8!",
  "side_to_move": "White"
}`;

    const url = `https://generativelanguage.googleapis.com/v1beta/models/gemini-2.5-flash:generateContent?key=${apiKey}`;
    const response = await fetch(url, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        contents: [{
          parts: [
            { text: promptText },
            { inline_data: { mime_type: "image/jpeg", data: base64Image } }
          ]
        }]
      })
    });

    if (!response.ok) {
      const err = await response.json();
      throw new Error(err.error?.message || "HTTP " + response.status);
    }

    const data = await response.json();
    let raw = data.candidates?.[0]?.content?.parts?.[0]?.text || "{}";
    raw = raw.replace(/```json/g, '').replace(/```/g, '').trim();
    return JSON.parse(raw);
  }

  /**
   * 👀 "What do you see?" — one playful, kid-safe sentence about the scene.
   *
   * TEXT ONLY, BY DESIGN. Unlike analyzeVisionTarget(), this returns a plain
   * string and never an actions array, so nothing it says can ever become
   * robot motion. It is a describer, not a driver.
   */
  async function describeScene(base64Image) {
    const apiKey = getApiKey();
    if (!apiKey || apiKey.length < 10) {
      throw new Error("Gemini API key not configured. Tap the badge to set your key.");
    }

    const promptText = `You are S2-R2-D2, a friendly astromech droid talking to an 8-year-old child.
Look at this photo and say what you see in ONE short, playful, cheerful sentence.
Rules: at most 90 characters. Plain English words only, no emoji, no markdown, no quotes.
Never mention people's names, faces, appearance, or anything unkind or scary.
If you are unsure what something is, say so happily and guess.
Reply with ONLY that one sentence and nothing else.`;

    const url = `https://generativelanguage.googleapis.com/v1beta/models/gemini-2.5-flash:generateContent?key=${apiKey}`;
    const response = await fetch(url, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        contents: [{
          parts: [
            { text: promptText },
            { inline_data: { mime_type: "image/jpeg", data: base64Image } }
          ]
        }]
      })
    });

    if (!response.ok) {
      const err = await response.json();
      throw new Error(err.error?.message || "HTTP " + response.status);
    }

    const data = await response.json();
    const raw = data.candidates?.[0]?.content?.parts?.[0]?.text || "";
    // Flatten to a single clean sentence; never returns an object or actions.
    return String(raw).replace(/[`*_#]/g, '').replace(/\s+/g, ' ').trim().slice(0, 120);
  }

  return {
    getApiKey: getApiKey,
    setApiKey: setApiKey,
    hasApiKey: hasApiKey,
    fetchLiveWeather: fetchLiveWeather,
    fetchRoyalBriefing: fetchRoyalBriefing,
    askVocalBrain: askVocalBrain,
    analyzeVisionTarget: analyzeVisionTarget,
    analyzeChessboard: analyzeChessboard,
    describeScene: describeScene
  };
})();

if (typeof module !== 'undefined' && module.exports) {
  module.exports = CortexAPI;
}
