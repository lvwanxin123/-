import HomeClient from "@/components/HomeClient";
import {isChinesePrompt} from "@/lib/prompt-language";
import {createClient} from "@/lib/supabase/server";
import type {Prompt,UserFavorite} from "@/types";
export const dynamic="force-dynamic";
export default async function Home(){let prompts:Prompt[]=[];let favoritePromptIds:string[]=[];let loadError:string|null=null;try{const supabase=createClient();const {data,error}=await supabase.from("prompts").select("*").order("created_at",{ascending:false}).range(0,2999);if(error)throw error;prompts=((data??[]) as Prompt[]).filter(isChinesePrompt);const {data:{user}}=await supabase.auth.getUser();if(user){const {data:favorites}=await supabase.from("user_favorites").select("prompt_id").eq("user_id",user.id);favoritePromptIds=((favorites??[]) as Pick<UserFavorite,"prompt_id">[]).map(f=>f.prompt_id)}}catch(error){console.error(error);loadError="提示词加载失败，请检查 Supabase 配置或稍后刷新。"}return <HomeClient prompts={prompts} favoritePromptIds={favoritePromptIds} loadError={loadError}/>}
